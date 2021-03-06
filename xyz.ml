(* Float coords 3D --------------------------------------------------------- *)

(* Type of a point or a vector *)
type point = {                    	
  x: float;                 		(* Position x *)
  y: float;                 		(* Position y *)
  z: float;             				(* Altitude *)
  zs: float;				(* Altitude selected *)
}


(* Basic operations on points/vectors *)
let mul k p = {x=k*.p.x; y=k*.p.y; z=k*.p.z; zs=k*.p.zs;}						(* Multiplication of coordinates by a scalar number*)
let add p1 p2 = {x=p1.x+.p2.x; y=p1.y+.p2.y; z=p1.z+.p2.z; zs=p1.zs+.p2.zs;}		(* Addition coordinate to coordinate *)
let sub p1 p2 = {x=p1.x-.p2.x; y=p1.y-.p2.y; z=p1.z-.p2.z; zs=p1.zs-.p2.zs;}		(* Substraction coordinate to coordinate *)

let sca p1 p2 = p1.x *. p2.x +. p1.y *. p2.y 					(* Scalar product Oxy*)
let det p1 p2 = p1.x *. p2.y -. p1.y *. p2.x 					(* Determinant Oxy *)

(* Euclidian Norm Oxy *)
let norm2 v = sca v v (* Norm² *)
let norm v = sqrt (norm2 v) (* Norm *)

let sepxy p1 p2 xysep = 
	let dx = (p1.x -. p2.x) in
	let dy = (p1.y -. p2.y) in
	let dxy = dx*.dx +. dy*.dy in
	dxy/.(xysep*.xysep)
	
let sepz p1 p2 hsep = 
	let dz = (p1.z -. p2.z) in
	(dz*.dz)/.(hsep*.hsep)
	
let sep2 p1 p2 xysep hsep = 
	sepxy p1 p2 xysep +. sepz p1 p2 hsep


(* Changes altitude *)

let change_alt p alt = {x=p.x; y=p.y; z= alt; zs = p.zs}
let change_alts p alts = {x=p.x; y=p.y; z= p.z; zs = alts}
let change_all p alt alts = {x=p.x; y=p.y; z= alt; zs = alts}

(* Barycentre of two points *)
let bary (t1, p1) (t2, p2) t =
  add p1 (mul ((t -. t1) /.(t2 -. t1)) (sub p2 p1))
  

(* Conversions between angles *)
let pi = atan2 0. (-1.) (* definition of pi *)
let twopi = 2. *. pi (* definition of 2*pi *)
let radians = pi /. 180. (* rate to convert degrees into radians *)
let degrees = 180. /. pi (* rate to convert radians into degrees *)

(* Arctan of y/x, gives theta angle for polar conversion Oxy*)
let angle p = atan2 p.y p.x

let angle2 p1 p2 p3= 
	let pa = sub p2 p1 in
	let pb = sub p3 p2 in
	degrees *. (acos (-.(sca pa pb) /. (norm pa *. norm pb)))
	

(*Polar coordinates*)
let polar a = (cos a, sin a)

(*modulo 2 Pi*)		
let mod_twopi a =
  let b = mod_float a twopi in
  if pi <= b then b -. twopi else if b < -.pi then b +. twopi else b

(*Angle addition*)						     
let add_angle a1 a2 = mod_twopi (a1 +. a2)

(*Angle substraction*)				
let sub_angle a1 a2 = mod_twopi (a1 -. a2)

(*Distance between two segments*)
let dist_seg_seg (a, b) (c, d) =
  let dseg ap ab = (* distance (p, [ab]) *)
    let bp = sub ap ab in
    if sca ab ap < 0. then norm ap
    else if 0. < sca ab bp then norm bp
    else abs_float (det ab ap) /.norm ab in
  let ckw u v = (* orientation (u, v) *)
    let d = det u v in
    if 0. < d then 1 else if d < 0. then (-1) else (
      let uv = sca u v in
      if uv < 0. then (-1) else if uv <= sca u u then 0 else 1) in
  let ab = sub b a and ac = sub c a and ad = sub d a in
  let cd = sub d c and ca = sub a c and cb = sub b c in
  if ckw ab ac * ckw ab ad <= 0 && ckw cd ca * ckw cd cb <= 0 then 0.
  else min (min (dseg ca cd) (dseg cb cd)) (min (dseg ac ab) (dseg ad ab))

(*test*)

let xy2xyz (x,y) = {x=x;y=y;z=0.;zs=0.;}


let truncate n = 
	let i =  float_of_int (int_of_float n) in
	let j = float_of_int (int_of_float (n+.1.)) in
	if (n-.i) < (j-.n) then int_of_float i
	else int_of_float j
