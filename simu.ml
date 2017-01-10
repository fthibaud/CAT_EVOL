let max_n = 5            (* Max number of aircraft *)
let lag = 30.            (* Time lag before pilot action *)
let speedup = 1.         (* Initial time acceleration *)
let scale = 10.          (* Initial scale (pixels / Nm) *)
let size = 600.          (* Initial window size (pixels) *)
let mode = 2             (* Initial mode: Basic, Static, Dynamic *)


let hour = 3600.

let background = `Color "#468"     (* Background color *)
let scale_color = `Color "yellow"  (* Scale color *)
let acft_color = `Color "white"    (* Aircraft normal color *)
let conf_color = `Color "red"      (* Highlighted conflicts color *)
let conf_color2 = `Color "#234"    (* Other conflicts color *)
let pln_color = `Color "#68a"      (* Route color *)
let edit_color = `Color "cyan"     (* Highlighted route color *)

let scale_tag = "Scale"
let acft_tag = "Aircraft"
let around_tag = "Around"
let pln_tag = "PLN"
let edit_tag = "Edit"
let conf_tag = "Conflict"
let current_tag = "current"

let round a = truncate (floor (a +. 0.5))

type mode = Basic | Static | Dynamic | Show

type state = {
  cv: Widget.canvas Widget.widget;
  btn : Widget.button Widget.widget array;
  mutable acft: Acft.t array;   (* Aircraft array *)
  mutable mode: mode;           (* Visu mode *)
  mutable cur: int;             (* Mouse current aircraft index or -1 *)
  mutable dev_xy: Xyz.point array;   (* Mouse world coords during drag or [||] *)
  mutable timer: Timer.t;       (* Animation loop timer *)
  mutable start_t: float;       (* Absolute beginning time in sec. *)
  mutable speedup: float;       (* Time acceleration *)
  mutable scale: float;         (* pixels per Nm *)
  mutable xy0: Xyz.point;            (* pixels *)
}

let time = Unix.gettimeofday

let rtime state = time () -. state.start_t

let state_time state = state.speedup *. rtime state
  
let new_state state n =
  Canvas.delete state.cv [`Tag "all"];
  let w = float (Winfo.width state.cv) in
  let h = float (Winfo.height state.cv) in
  state.xy0 <- Xyz.mul 0.5 Xyz.{x=w; y=h; z=0.;zs=0.};
  state.acft <- Acft.roundabout (min w h /. scale /. 2.) n;
  let times = Array.map Acft.t_start state.acft in
  Array.sort compare times;
  state.start_t <- time () -. times.(1);
  state.cur <- -1;
  state.dev_xy <- [||];
  state.speedup <- 1.

(* Drawings ---------------------------------------------------------------- *)

let coords_get cv item =
  let rec get = function x::y::tl -> (round x, round y)::get tl | _ -> [] in
  get (Canvas.coords_get cv item)

let cv_xy state xy =
  let p = Xyz.add state.xy0 (Xyz.mul state.scale xy) in
  (round p.Xyz.x,round p.Xyz.y)

let world_xy state (x, y) =
  Xyz.mul (1. /. state.scale) (Xyz.sub Xyz.{x= float x; y= float y;z=0.;zs=0.} state.xy0)


let tag_id tag id =
  Printf.sprintf "%s %d" tag id

let get_id = function
  | _::ti::_ -> (try Scanf.sscanf ti "%s %d" (fun tag id -> id) with _ -> -1)
  | _ -> -1

let t_dev a = max (Acft.t_start a) (Acft.t_cur a) +. lag

let draw_scale state =
  Canvas.delete state.cv [`Tag scale_tag];
  let x = 10 + round (Acft.sep *. state.scale) in
  ignore (Canvas.create_line ~xys:[(10, x); (10, 10); (x, 10)]
    ~fill:scale_color ~arrow:`Both ~tags:[scale_tag] state.cv);
  ignore (Canvas.create_text ~x:20 ~y:20 
	    ~text:(Printf.sprintf "%.0fNm" Acft.sep)
	    ~fill:scale_color ~anchor:`Nw ~tags:[scale_tag] state.cv)

let draw_pln state id =
  let pln = state.acft.(id).Acft.pln in
  let xys = Array.to_list (Array.map (fun (_, xy) -> cv_xy state xy) pln) in
  let tags = [around_tag; tag_id around_tag id] in
  Canvas.delete state.cv [`Tag (List.hd (List.tl tags))];
  ignore (Canvas.create_line ~xys:xys 
	    ~fill:(`Color "") ~width:15 ~tags:tags state.cv);
  let tags = [pln_tag; tag_id pln_tag id] in
  Canvas.delete state.cv [`Tag (List.hd (List.tl tags))];
  ignore (Canvas.create_line ~xys:xys ~fill:pln_color 
	    ~arrow:`Last ~tags:tags state.cv);
  Canvas.lower state.cv (`Tag pln_tag)

let draw_dev state dev =
  Canvas.delete state.cv [`Tag edit_tag];
  let xys = Array.to_list (Array.map (fun (_, xy) -> cv_xy state xy) dev) in
  ignore (Canvas.create_line ~xys:xys
	    ~fill:edit_color ~tags:[edit_tag] state.cv);
  Canvas.raise state.cv (`Tag conf_tag)

let draw_acft state =
  Canvas.delete state.cv [`Tag acft_tag];
  let conf = Acft.pos_detect state.acft in
  Array.iteri (fun i acft ->
    let tags = [acft_tag; tag_id acft_tag i] in
    let next_pos = Acft.get_vector acft in
    let color = if conf.(i) then conf_color else acft_color in
    let labeltext = ref "" in
    let t = state_time state in
    let pos =  Xyz.bary (acft.Acft.pln.(acft.Acft.leg)) (acft.Acft.pln.(min (acft.Acft.leg+1) (Array.length acft.Acft.pln - 1))) t in
    let lvl = pos.Xyz.z in
    let lvls = pos.Xyz.zs in
	if (lvl = lvls) then labeltext :=  String.concat "" ["FL";(string_of_int (Xyz.truncate lvl))];
	if (lvl > lvls) then labeltext :=  String.concat "" ["FL";(string_of_int (Xyz.truncate lvl));"↘\nCFL";(string_of_int (Xyz.truncate lvls))];
	if (lvl < lvls) then labeltext :=  String.concat "" ["FL";(string_of_int (Xyz.truncate lvl));"↗\nCFL";(string_of_int (Xyz.truncate lvls))];
   (* Plot *)

    let (x, y as xy) = cv_xy state (Acft.get_pos acft) and d = 5 in
    ignore (Canvas.create_rectangle ~x1:(x-d) ~y1:(y-d) ~x2:(x+d) ~y2:(y+d)
	      ~outline:color ~tags:tags state.cv);
    (* Speed vector *)
    ignore (Canvas.create_line ~xys:[xy; cv_xy state next_pos]
	      ~fill:color ~width:2 ~tags:tags state.cv);
   (* Label *)
     ignore (Canvas.create_text ~x:(x+10) ~y:(y-10)
		~text: (!labeltext)
	    ~fill:scale_color ~anchor:`Nw ~tags:tags state.cv);
    (* Comet *)
    Array.iteri (fun j xyj ->
      let (x, y) = cv_xy state xyj and d = 4 - j in
      ignore (Canvas.create_oval ~x1:(x-d) ~y1:(y-d) ~x2:(x+d) ~y2:(y+d)
		~outline:color ~tags:tags state.cv)) (Acft.get_comet acft))
    state.acft

let draw_conf state =
  let draw_segs i j segs =
    let tags = [conf_tag; tag_id conf_tag i; tag_id conf_tag j] in
    List.iter (fun (xy1, xy2) ->
      ignore (Canvas.create_line ~xys:[cv_xy state xy1; cv_xy state xy2]
		~fill:conf_color2 ~width:2 ~tags:tags state.cv)) segs in
  Canvas.delete state.cv [`Tag conf_tag];
  let t = Acft.t_cur state.acft.(0) in
  Array.iteri (fun i ai ->
    if t < Acft.t_end ai then
      for j = 0 to i - 1 do
	if t < Acft.t_end state.acft.(j) then (
	  let (segsi, segsj) = snd (Acft.detect ai state.acft.(j)) in
	  draw_segs i j segsi;
	  draw_segs j i segsj);
      done) state.acft;
  if state.cur <> -1 then (
    let items = `Tag (tag_id conf_tag state.cur) in
    Canvas.configure_line ~fill:conf_color state.cv items;
    Canvas.raise state.cv items)

let draw_all state =
  if Canvas.gettags state.cv (`Tag scale_tag) = [] then draw_scale state;
  if state.mode = Basic then Canvas.delete state.cv [`Tag conf_tag];
  let t = state_time state in
  Array.iteri (fun id acft ->
    Acft.update acft t;
    if state.mode = Dynamic && state.cur = id && state.dev_xy <> [||] then (
      acft.Acft.predict <- Acft.dev acft (t_dev acft) state.dev_xy.(0));
    if Canvas.gettags state.cv (`Tag (tag_id pln_tag id)) = [] then
      draw_pln state id) state.acft;
  draw_acft state;
  if state.mode <> Show then (
    if state.cur <> -1 && state.dev_xy <> [||] then (
      let a = state.acft.(state.cur) in
      let dev = Acft.dev a (t_dev a) state.dev_xy.(0) in
      draw_dev state dev;
      if state.mode = Dynamic then state.acft.(state.cur).Acft.predict <- dev);
    if state.mode <> Basic then draw_conf state)

(* Interactions ----------------------------------------------------------- *)

let highlight_current state evnt =
  if state.mode <> Show then (
    state.cur <- get_id (Canvas.gettags state.cv (`Tag current_tag));
    Canvas.configure_line ~fill:pln_color state.cv (`Tag pln_tag);
    Canvas.configure_line ~fill:conf_color2 state.cv (`Tag conf_tag);
    if state.cur <> -1 then (
      let tag = `Tag (tag_id around_tag state.cur) in
      Canvas.raise state.cv ~above:(`Tag around_tag) tag;
      let tag = `Tag (tag_id pln_tag state.cur) in
      Canvas.configure_line ~fill:edit_color state.cv tag;
      if state.mode <> Basic then draw_conf state))

let drag_edit state evnt =
  if state.mode <> Show && state.cur <> -1 then (
    let a = state.acft.(state.cur) in
    let t = state_time state in
    if t < Acft.t_end a -. lag then (
      if state.dev_xy = [||] then (
	Canvas.configure_line ~fill:pln_color state.cv (`Tag pln_tag));
      let mouse_xy = (evnt.Tk.ev_MouseX, evnt.Tk.ev_MouseY) in
      state.dev_xy <- [|world_xy state mouse_xy|];
      let dev = Acft.dev a (t_dev a) state.dev_xy.(0) in
      draw_dev state dev;
      if state.mode = Dynamic then (
	a.Acft.predict <- dev;
	draw_conf state))
    else (
      state.dev_xy <- [||];
      Canvas.delete state.cv [`Tag edit_tag];
      Acft.update a (Acft.t_cur a);
      if state.mode = Dynamic then draw_conf state))

let apply_edit state evnt =
  if state.mode <> Show && state.cur <> -1 && state.dev_xy <> [||] then (
    let a = state.acft.(state.cur) in
    a.Acft.predict <- Acft.dev a (t_dev a) state.dev_xy.(0);
    Acft.apply_dev a;
    state.dev_xy <- [||];
    let dt = Array.fold_left (fun dt acft -> 
      dt +. Acft.delay acft) 0. state.acft in
    Printf.printf "delay = %.0f\n%!" dt;
    let tag = `Tag (tag_id pln_tag state.cur) in
    Canvas.delete state.cv [tag; `Tag edit_tag];
    draw_all state;
    Canvas.configure_line ~fill:edit_color state.cv tag)
    
let sel_lvl_edit state evnt =
  if state.mode <> Show then
    Printf.printf "Scroll"
		

let cancel_edit state =
  if state.mode <> Show then (
    Canvas.delete state.cv [`Tag edit_tag];
    Canvas.configure_line ~fill:pln_color state.cv (`Tag pln_tag);
    let a = state.acft.(state.cur) in
    state.cur <- -1;
    state.dev_xy <- [||];
    if state.mode = Dynamic then (
      Acft.update a (Acft.t_cur a);
      draw_conf state))
    
let incr_speed state dspeed () =
  let t = time () in
  let old = state.speedup in
  state.speedup <- 
    if dspeed = 0. then 1. else (max 1. (state.speedup +. dspeed));
  state.start_t <- t +. old /.state.speedup *. (state.start_t -. t)

let reset state () =
  new_state state (2 + Random.int (max_n - 1));
  incr_speed state (speedup -. state.speedup) ();
  draw_all state

let rec loop state next_action () =
  Timer.remove state.timer;
  let ms = round (1000. /.state.speedup) in
  state.timer <- Timer.add ms (loop state next_action);
  let t = state_time state in
  if t < Array.fold_left max 0. (Array.map Acft.t_end state.acft) then (
    if state.cur <> -1 && Acft.t_end state.acft.(state.cur) -. lag < t then (
      state.dev_xy <- [||]);
    draw_all state)
  else (
    reset state (); 
    next_action ())

let next state next_action () =
  let dt = Array.fold_left (fun dt a -> dt +. Acft.delay a) 0. state.acft in
  Printf.printf "delay = %.0f\n%!" dt;
  incr_speed state (250. -. state.speedup) ();
  loop state next_action ()

let redo state () =
  Array.iter Acft.reset state.acft;
  let times = Array.map Acft.t_start state.acft in
  Array.sort compare times;
  state.start_t <- time () -. times.(1);
  state.cur <- -1;
  state.dev_xy <- [||];
  Canvas.delete state.cv [`Tag "all"];
  draw_all state;
  loop state (reset state) ()

let scroll state evnt =
  state.start_t <- state.start_t -. Acft.delta /.2.;
  draw_all state

let back state evnt =
  state.start_t <- state.start_t +. Acft.delta;
  draw_all state
  
let forward state evnt =
  state.start_t <- state.start_t -. Acft.delta;
  draw_all state

let change_mode state =
  let text = match state.mode with
    | Basic -> state.mode <- Static; "Mode: Static"
    | Static -> state.mode <- Dynamic; "Mode: Dynamic"
    | Dynamic -> state.mode <- Basic; "Mode: Basic"
    | _ -> "" in
  if text <> "" then (
    Button.configure ~text:text state.btn.(0);
    draw_all state)

let main =
  (* Set current directory *)
  Sys.chdir (Filename.dirname Sys.executable_name);

  (* Build the GUI *)
  let top = Tk.openTk() in
  Wm.title_set top "Control Assistance Tool";
  let frame = Frame.create top in
  let width = round size in
  let state = {
    cv = Canvas.create ~width:width ~height:width ~background:background top;
    btn = Array.map (fun text -> Button.create ~text:text frame)
      [|"Mode: " ^ [|"Basic"; "Static"; "Dynamic"|].(mode); "Redo"; "Next"|];
    acft = [||];
    mode = [|Basic; Static; Dynamic|].(mode);
    cur = -1;
    dev_xy = [||];
    timer = Timer.add 0 (fun () -> ());
    start_t = time ();
    speedup = speedup;
    scale = scale;
    xy0 = Xyz.mul 0.5 Xyz.{x= size; y=size; z=180.;zs=180.};

  } in
  Button.configure ~command:(fun () -> change_mode state) state.btn.(0);
  Button.configure ~command:(redo state) state.btn.(1);
  Button.configure ~command:(next state (fun () -> ())) state.btn.(2);
  Tk.pack ~expand:true ~fill:`Both [state.cv];
  Tk.pack ~side:`Left (Array.to_list state.btn);
  Tk.pack [frame];

  (* Bindings *)
  let bind events fields action w =
    Tk.bind ~events:events ~fields:fields ~action:action w in
  let mouse_xy = [`MouseX; `MouseY] in
  bind [`KeyPressDetail "q"] [] (fun _ -> Tk.closeTk ()) state.cv;
  bind [`KeyPressDetail "b"] [] (fun _ -> back state ()) state.cv;
  bind [`Motion] [] (highlight_current state) state.cv;
  bind [`ButtonPressDetail 1] mouse_xy (drag_edit state) state.cv;
  bind [`Modified ([`Button1], `Motion)] mouse_xy (drag_edit state) state.cv;
  bind [`ButtonReleaseDetail 1] [] (apply_edit state) state.cv;
  bind [`ButtonPressDetail 3] [] (fun _ -> cancel_edit state) state.cv;
  bind [`ButtonPressDetail 5] [] (scroll state) top;
  bind [`KeyPressDetail "f"] [] (fun _ -> forward state ())(*(sel_lvl_edit state)*) state.cv;
  bind [`KeyPressDetail "p"] [] (sel_lvl_edit state) state.cv;

 
  Focus.set state.cv;

  (* Go *)
  Tk.update();
  new_state state 2;
  loop state (reset state) ();
  Tk.mainLoop ()
