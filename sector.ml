let waypoints = Hashtbl.create 7;;
Hashtbl.replace waypoints "TOU1" (Xyz.{x=(-.22.); y=(-.23.); z=0.;zs=0.},false);
Hashtbl.replace waypoints "TOU2" (Xyz.{x=21.; y=(-.19.); z=0.;zs=0.},false);
Hashtbl.replace waypoints "TOU3" (Xyz.{x=20.; y=24.; z=0.;zs=0.},false);
Hashtbl.replace waypoints "TOU4" (Xyz.{x=(-.26.); y=20.; z=0.;zs=0.},false);
Hashtbl.replace waypoints "TOU5" (Xyz.{x=(-.9.); y=(-.2.); z=0.;zs=0.},true);
Hashtbl.replace waypoints "TOU6" (Xyz.{x=2.; y=(-.9.); z=0.;zs=0.},true);
Hashtbl.replace waypoints "TOU7" (Xyz.{x=0.5; y=15.; z=0.;zs=0.},true);;


let plns = [|
		[|"TOU1";"TOU5";"TOU7";"TOU3"|];
		[|"TOU1";"TOU6";"TOU3"|];
		[|"TOU3";"TOU7";"TOU5";"TOU1"|];
		[|"TOU3";"TOU6";"TOU1"|];
		[|"TOU2";"TOU6";"TOU5";"TOU4"|];
		[|"TOU2";"TOU7";"TOU4"|];
		[|"TOU4";"TOU7";"TOU2"|];
		[|"TOU4";"TOU5";"TOU6";"TOU2"|];
		
	|];;

let getpln alt =
	let rand = Random.int (Array.length plns) in
	let pln = plns.(rand) in
	let apln = Array.init (Array.length pln) (fun i ->
										Xyz.change_all (fst (Hashtbl.find waypoints pln.(i))) alt alt;
									) in
	Array.to_list apln
									
	
