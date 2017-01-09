(* ocamlopt.opt -o simu -I +labltk labltk.cmxa unix.cmxa simu.ml *)

let delta = 5.           			(* trace time steps (sec) *)
let vector = 60.         			(* speed vector size (sec) *)
let max_speed = 550.     			(* max speed (Knots) *)
let vertical_speed = 10./.60. 		(* vertical speed (FL/sec) *)
let dspeed = 0.05        			(* Speed uncertainty rate *)
let alpha = 5.           (* turn angle discretisation (deg) *)
let std_turn = 120.      (* time for a full standard round (sec) *)
let max_turn = 15.       (* max turn (deg) *)
let sep = 5.           (* minimal vertical separation (Nm) *)
let hsep = 10.          (* minimal vertical separation (Nm) *)
let max_start_t = 30.    (* maximam start time (sec) *)
let min_conf_t = 30.     (* minimal time of first conflict *)

let hour = 3600.

let round a = truncate (floor (a +. 0.5))

type time = float  (* Seconds *)

type speed = float  (* Knots *)

type pln = (float * Xyz.point) array (* [|...(time in seconds, coords in Nm)...|] *)

type t = {
speed: float;                     (* Knots *)
vspeed: float;					  (* Vertical speed (FL/sec) *)
mutable pln: pln;                 (* Real flightplan *)
mutable leg: int;                 (* Current leg in flightplan *)
mutable predict: pln;             (* Prediction *)
mutable route: Xyz.point array;   (* Positions every delta sec. *)
}
	

let pln_to_string pln =
	String.concat " " (Array.to_list (Array.map (fun (t, (x, y)) ->
						 Printf.sprintf "%.2f,%.3f,%.3f" t x y) pln))

let pln_of_string s =
	let rec parse = function
		| "" -> []
		| s -> Scanf.sscanf s "%f,%f,%f %[^\n]" (fun t x y tl -> 
						 (t, (x, y))::parse tl) in
Array.of_list (parse s)

let create_pln speed ds t0 xys =
	let t = ref t0 in
	Array.mapi (fun i xyi ->
		if i <> 0 then (
				let dt = hour /. speed /. (1. +. Random.float (2.*. ds) -. ds) in
				    t := !t +. Xyz.norm (Xyz.sub xyi xys.(i - 1)) *. dt);
		(!t, xyi)) xys

let create_route pln =
	let rec get t i =
		if i = Array.length pln then []
		else if t < fst pln.(i) then 
				Xyz.bary pln.(i - 1) pln.(i) t::get (t +. delta) i
			 else get t (i + 1)
	in
	Array.of_list (get (fst pln.(0)) 1)


let t_start acft = fst acft.pln.(0)

let t_cur acft = fst acft.predict.(0)

let t_end acft = fst acft.pln.(Array.length acft.pln - 1)

let update acft t =
	let last = Array.length acft.pln - 1 in
	if t <= fst acft.pln.(acft.leg) then acft.leg <- 0;
	while acft.leg < last && fst acft.pln.(acft.leg + 1) < t do
		acft.leg <- acft.leg + 1
	done;
	let xys = Array.init (Array.length acft.pln - acft.leg) (fun i ->
							     snd acft.pln.(acft.leg + i)) in
	let i = min (Array.length acft.pln - 2) acft.leg in
	if t < fst acft.pln.(0) then (
		let d = Xyz.norm (Xyz.sub (snd acft.pln.(i + 1)) (snd acft.pln.(i))) in
		let t1 = fst acft.pln.(i) +. d *. hour /. acft.speed in
		xys.(0) <- Xyz.bary acft.pln.(i) (t1, snd acft.pln.(i + 1)) t)
	else xys.(0) <- Xyz.bary acft.pln.(i) acft.pln.(i + 1) t;
	acft.predict <- create_pln acft.speed 0. t xys
		
	

let get_pos acft =
	snd acft.predict.(0)

let get_vector acft =
	let t = fst acft.predict.(0) in
	let i = min (Array.length acft.pln - 2) acft.leg in
	Xyz.bary acft.pln.(i) acft.pln.(i + 1) (t +. vector)

let get_comet acft =
	let t = fst acft.predict.(0) in
	let i0 = truncate (floor ((t -. fst acft.pln.(0)) /. delta)) in
	let i = max 0 (min (Array.length acft.route - 2) i0) in
	let p1 = (float i, acft.route.(i)) in
	let p2 = (float (i + 1), acft.route.(i + 1)) in
	Array.init 5 (fun j ->
		  let k = i0 - j in
		  if 0 <= k && k < Array.length acft.route then acft.route.(k) 
		  else Xyz.bary p1 p2 (float k))

let reset acft =
	let (t,xyz0) = acft.pln.(0) in
	let last = Array.length acft.pln - 1 in
	let (tf, xyzf) = acft.pln.(last) in
	let lvl = xyz0.Xyz.z in
	let xys = [|(Xyz.change_alts xyz0 lvl); (Xyz.change_all xyzf lvl lvl)|] in
	acft.pln <- create_pln acft.speed dspeed t xys;
	acft.leg <- 0;
	acft.predict <- create_pln acft.speed 0. t xys;
	acft.route <- create_route acft.pln

let nav speed xys =
	let max_a = max_turn *. Xyz.radians in
	let da = alpha *. Xyz.radians in
	let epsilon = da /. 2. in
	let radius = speed /. hour *. std_turn /. Xyz.pi /. 2. in
	let min_d = radius *. max_a in
	let rec get hdg = function
	| xy1::xy2::tl when xy1 = xy2 -> get hdg (xy1::tl)
    | xy1::xy2::tl ->
		let v = Xyz.sub xy2 xy1 in
		let d = Xyz.norm v in
		let dir = Xyz.angle v in
		let keep_xy2 = match tl with
			| [xy] when Xyz.norm (Xyz.sub xy1 xy) < d +. 1. *. radius -> false
			| xy::_ -> 
				epsilon < abs_float (Xyz.sub_angle (Xyz.angle (Xyz.sub xy xy2)) dir)
			| _ -> true in
		if keep_xy2 then (
			let turn = Xyz.sub_angle dir hdg in
			if max_a +. epsilon < abs_float turn then (
				let h = Xyz.sub_angle hdg (if turn < 0. then max_a else -. max_a) in
				let xy = Xyz.add xy1 (Xyz.mul min_d Xyz.{x=cos h; y=sin h;z=0.;zs=0.}) in
				let d2 = Xyz.norm (Xyz.sub xy xy2) in
				xy1::get h (xy::if radius < d2 || tl = [] then xy2::tl else tl))
			else (
				let a = 
					if tl = [] then turn else (da *. float (round (turn /. da))) in
				let h = Xyz.sub_angle hdg (-.a) in
				xy1::get h (Xyz.add xy1 (Xyz.mul (max d min_d) Xyz.{x=cos h; y=sin h;z=0.;zs=0.})::tl)
				)
		)
      else get hdg (xy1::tl)
    | l -> l in
	get (Xyz.angle (Xyz.sub (List.nth xys 1) (List.hd xys))) xys

let dev_lvl acft t_dev xy_dev lvl_dev =
	let (t, xyt) = acft.predict.(0) in
	let leg = ref acft.leg and last = Array.length acft.pln - 1 in
	while !leg < last && fst acft.pln.(!leg + 1) <= t_dev do incr leg done;
	let leg1 = !leg in
	
	let deltalvl = lvl_dev -. xyt.Xyz.z in
	let t_end = t_dev +. deltalvl /. acft.vspeed in
	while !leg < last && fst acft.pln.(!leg + 1) <= t_end do incr leg done;
	let leg2 = !leg in
	let first = Array.init (leg1 - acft.leg + 1) (fun j -> 
					      snd acft.pln.(acft.leg + j)) in
					      
	let p_start = Xyz.change_alts (Xyz.change_alt xy_dev xyt.Xyz.z) lvl_dev in
	let p_end = Xyz.change_alts (Xyz.change_alt (Xyz.bary (acft.pln.(leg2)) (acft.pln.(leg2+1)) t_end) lvl_dev) lvl_dev in
	
	let middle = Array.init (leg2 - leg1) (fun j -> 
						  let current = acft.pln.(leg1 + j) in
						  let deltat = (fst current) -. t_dev in
					      Xyz.change_alts (Xyz.change_alt (snd current) ((snd current).Xyz.z +. deltat *. acft.vspeed)) lvl_dev
					      ) in
					      
	let next = Array.init (last - leg2) (fun j -> 
						  let current = acft.pln.(leg2 + j + 1) in
					      Xyz.change_alts (Xyz.change_alt (snd current) lvl_dev) lvl_dev
					      ) in
	let list = [first; [|p_start|]; middle; [|p_end|]; next] in
	create_pln acft.speed 0. t (Array.concat list)


	
let dev acft t_dev xy_dev =
	let (t, xyt) = acft.predict.(0) in
	let leg = ref acft.leg and last = Array.length acft.pln - 2 in
	while !leg < last && fst acft.pln.(!leg + 1) <= t_dev do incr leg done;
	let first = Array.init (!leg - acft.leg) (fun j -> 
					      snd acft.pln.(acft.leg + j)) in
	if acft.leg < !leg then first.(0) <- xyt;
	let xy0 = if !leg = acft.leg then xyt else snd acft.pln.(!leg) in
	let xyd = Xyz.bary acft.pln.(!leg) acft.pln.(!leg + 1) t_dev in
	let xye = snd acft.pln.(last + 1) in	
	let next = Array.of_list (nav acft.speed [xy0; xyd; xy_dev; xye]) in
	let predict = create_pln acft.speed 0. t (Array.append first next) in
	let target = xyd.Xyz.zs in
	let deltah = float_of_int(abs (int_of_float (xyd.Xyz.z -. target))) in
	if deltah <> 0. then (
		let deltat = deltah /. acft.vspeed in
		let t_target = t_dev +. deltat in
		let next2 = ref [||] in
		let ptoc = ref true in
		Array.iteri (fun i (t,xyz) ->
				if i = 0 then next2 := Array.append !next2 [|(t,xyz)|]
				else (
					let (tlast,last) = !next2.(i-1) in
					if (t < t_target) then (
						let xyz2 = Xyz.change_all xyz (last.Xyz.z +. acft.vspeed *. (t-.tlast)) target in
						next2 := Array.append !next2 [|(t,xyz2)|]
					)
					else (
					if !ptoc then (
						let toc = Xyz.change_all (Xyz.bary (tlast,last) (t,xyz) t_target) target target in
						next2 := Array.append !next2 [|(t_target,toc)|];
						ptoc := false
						);
					let xyz2 = Xyz.change_all xyz target target in
						next2 := Array.append !next2 [|(t,xyz2)|]
					);
			);
		) predict;
		!next2
	)
	else predict



let apply_dev acft =
	let xys = Array.map snd acft.predict in
	if fst acft.predict.(0) < fst acft.pln.(0) then (
		xys.(0) <- snd acft.pln.(0);
		acft.pln <- create_pln acft.speed dspeed (fst acft.pln.(0)) xys)
	else (
		let next = create_pln acft.speed dspeed (fst acft.predict.(0)) xys in
		acft.pln <- Array.concat [Array.sub acft.pln 0 (acft.leg + 1); next]);
	acft.route <- create_route acft.pln		

	  
let delay acft =
	let l = Array.length acft.pln in
	let d_min = Xyz.norm (Xyz.sub (snd acft.pln.(l - 1)) (snd acft.pln.(0))) in
	let d = ref 0. in
	for i = 0 to l - 2 do 
		d := !d +. Xyz.norm (Xyz.sub (snd acft.pln.(i + 1)) (snd acft.pln.(i)));
	done;
	(!d -. d_min) *. hour /. acft.speed

	  (* Conflicts detection ---------------------------------------------------- *)

let pos_detect acft =
	let conf = Array.map (fun _ -> false) acft in
	Array.iteri (fun i acfti ->
	    for j = 0 to i - 1 do
	    let posi = get_pos acfti in
	    let posj = get_pos acft.(j) in
	    if (Xyz.sep2 posi posj sep hsep < 1.)then 
	    (
	     conf.(i) <- true;
	     conf.(j) <- true)
	    done) acft;
	    conf
	    
let roots1 a b =
	(* solve ax + b < 0. in [0, 1] *)
	if a = 0. then (if b < 0. then [0., 1.] else [])
	else (
		let x = -. b /. a in
		if a < 0. then (if x < 1. then [max 0. x, 1.] else [])
		else if 0. < x then [0., min 1. x] else [])

let roots2 a b c =
(* solve ax2 + 2bx + c < 0. for x in [0, 1] *)
	if a = 0. then roots1 (2.*. b) c
	else (
      let delta' = b ** 2. -. a *. c in
	  if 0. < delta' then (
			       let r = sqrt delta' in
				   let x1 = (-.b -. r) /. a and x2 = (-.b +. r) /. a in
				   if a < 0. then (
						   if x1 < 1. then (
								    if 0. < x2 then [(0., x2); (x1, 1.)]
								       else [max 0. x1, 1.])
						      else if 0. < x2 then [0., min 1. x2] 
						      else [])
				   else if x1 < 1. then (
							 if 0. < x2 then [max 0. x1, min 1. x2]
							    else [])
				   else [])
	  else if a < 0. then [0., 1.]
	  else [])
	  

type seg = Xyz.point * Xyz.point

let seg_detect orig1 dest1 orig2 dest2 sep dsep =
	let x = Xyz.sub orig2 orig1 in
	let v = Xyz.sub (Xyz.sub dest2 dest1) x in
	let a = Xyz.norm2 v -. dsep ** 2. in
	roots2 a (Xyz.sca x v -. sep *. dsep) (Xyz.norm2 x -. sep ** 2.)

let seg_detect2 orig1 dest1 orig2 dest2 sep dsep hsep dhsep =
	let p1 = Xyz.sub (Xyz.sub dest2 orig2) (Xyz.sub dest1 orig1) in
	let p2 = Xyz.sub orig2 orig1 in
	let p1x = p1.Xyz.x and p2x = p2.Xyz.x in
	let p1y = p1.Xyz.y and p2y = p2.Xyz.y in
	let p1z = p1.Xyz.z and p2z = p2.Xyz.z in
	let xysep2 = sep*.sep in
	let zsep2 = hsep*.hsep in
	let a = (p1x*.p1x +. p1y*.p1y -. dsep ** 2.)/.xysep2 +. (p1z*.p1z -. dhsep ** 2.)/.zsep2 in
	let b = (p1x*.p2x +. p1y*.p2y -. sep *. dsep)/.xysep2 +. (p1z*.p2z -. hsep *. dhsep)/.zsep2 in
	let c = (p2x*.p2x +. p2y*.p2y -. sep ** 2.)/.xysep2 +. (p2z*.p2z -. hsep ** 2.)/.zsep2 in
	roots2 a b c

let detect acft1 acft2 =

	let first_t = ref max_float and segs1 = ref [] and segs2 = ref [] in
	
	let vsep = dspeed *. (acft1.speed +. acft2.speed) /. hour in
	let vhsep = dspeed *. (acft1.vspeed +. acft2.vspeed) /. hour in
	
	let pred1 = acft1.predict and pred2 = acft2.predict in
	
	let rec detect leg1 orig1 leg2 orig2 t sep_t hsep_t =
		if leg1 < Array.length pred1 && leg2 < Array.length pred2 then(
			let t1 = fst pred1.(leg1) and t2 = fst pred2.(leg2) in
			let next_t = min t1 t2 in
			let (dest1, next_leg1) =
				if t1 = next_t then (snd pred1.(leg1), leg1 + 1) 
				else (Xyz.bary pred1.(leg1 - 1) pred1.(leg1) next_t, leg1) in
			let (dest2, next_leg2) =
				if t2 = next_t then (snd pred2.(leg2), leg2 + 1) 
				else (Xyz.bary pred2.(leg2 - 1) pred2.(leg2) next_t, leg2) in
			let next_sep = sep +. vsep *. (next_t -. fst pred1.(0)) in
			let next_hsep = hsep +. vhsep *. (next_t -. fst pred1.(0)) in
			detect next_leg1 dest1 next_leg2 dest2 next_t next_sep next_hsep;
			let bary1 = Xyz.bary (0., orig1) (1., dest1) in
			let bary2 = Xyz.bary (0., orig2) (1., dest2) in
			List.iter (fun (t1, t2) ->
				let seg1 = (bary1 t1, bary1 t2) and seg2 = (bary2 t1, bary2 t2) in
				first_t := min !first_t (t +. t1 *. (next_t -. t) -. fst pred1.(0));
				segs1 := seg1:: !segs1;
				segs2 := seg2:: !segs2
				) (seg_detect2 orig1 dest1 orig2 dest2 sep_t (next_sep -. sep_t) hsep_t (next_hsep -. hsep_t))
		) 
	in
	if fst pred1.(0) <> fst pred2.(0) then
	Printf.printf "detect: %f <> %f\n%!" (fst pred1.(0)) (fst pred2.(0));
	detect 1 (snd pred1.(0)) 1 (snd (pred2.(0))) (fst pred1.(0)) sep hsep;
	(!first_t, (!segs1, !segs2))

let roundabout size n =
let r = 0.9 *. size in
let scramble x dx = x +. Random.float (2. *. dx) -. dx in
let random_acft alpha =
	let randomalt = float_of_int (10*(8+(Random.int 10))) in
	let xy0 = Xyz.add (Xyz.mul r Xyz.{x=cos alpha; y=sin alpha;z=0.;zs = 0.}) Xyz.{x=scramble 0. sep;y= scramble 0. sep;z=randomalt; zs=randomalt} in
	let b = scramble alpha 0.5 in
	let xys = [|xy0; Xyz.mul (-.r) Xyz.{x=cos b; y=sin b;z=(randomalt)/.(-.r);zs=(randomalt)/.(-.r)}|] in
	let speed = max_speed *. (0.7 +. Random.float (0.3 -. dspeed)) in
	(* Pour tester avec des heures de debut differentes... *)
	let t = Random.float max_start_t in
	(* let t = 0. in *)
	let pln = create_pln speed dspeed t xys in
	let route = create_route pln in
	let a = {speed=speed; vspeed = vertical_speed; pln=pln; leg=0; predict=[||]; route=route;} in
	update a 0.;
	a in
let rec first_conf_t a i j =
if i = j then max_float
else min (fst (detect a.(i) a.(j))) (first_conf_t a (i + 1) j) in
let slices = max n (round (Xyz.pi *. size /. sep /. 1.5)) in
let s = float slices in
let alpha = Array.init slices (fun i -> 2.*. float i *. Xyz.pi /. s) in
Array.sort (fun _ _ -> Random.int 3 - 1) alpha;
let acft = Array.init n (fun i -> random_acft alpha.(i)) in
Array.iteri (fun i _ ->
		 while first_conf_t acft 0 i < fst acft.(i).pln.(0) +. min_conf_t do
		 acft.(i) <- random_acft alpha.(i)
		 done) acft;
let t0 = (fst acft.(0).pln.(0)) in
let z0 = (snd acft.(0).pln.(0)).Xyz.z in
acft.(0).predict <- dev_lvl acft.(0) (t0 +. 1.) (Xyz.bary (acft.(0).pln.(0)) (acft.(0).pln.(1)) (t0 +. 1.)) (z0 +. 20.);
apply_dev acft.(0);
acft
