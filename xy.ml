(* Float coords 2D --------------------------------------------------------- *)

(* Type of a point or a vector *)
type t = float * float

(* Basic operations on points/vectors *)
let mul k (x, y) = (k *. x, k *. y) (* Multiplication of coordinates by a scalar number*)
let add (x1, y1) (x2, y2) = (x1 +. x2, y1 +. y2) (* Addition coordinate to coordinate *)
let sub (x1, y1) (x2, y2) = (x1 -. x2, y1 -. y2) (* Substraction coordinate to coordinate *)
let sca (x1, y1) (x2, y2) = x1 *. x2 +. y1 *. y2 (* Scalar product *)
let det (x1, y1) (x2, y2) = x1 *. y2 -. y1 *. x2 (* determinant *)

(* Euclidian Norm *)
let norm2 v = sca v v (* Norm² *)
let norm v = sqrt (norm2 v) (* Norm *)

(* Barycentre of two points *)
let bary (t1, xy1) (t2, xy2) t =
  add xy1 (mul ((t -. t1) /.(t2 -. t1)) (sub xy2 xy1))

(* Conversions between angles *)
let pi = atan2 0. (-1.) (* definition of pi *)
let twopi = 2. *. pi (* definition of 2*pi *)
let radians = pi /. 180. (* rate to convert degrees into radians *)
let degrees = 180. /. pi (* rate to convert radians into degrees *)

(* Arctan of y/x, gives theta angle for polar conversion *)
let angle (x, y) = atan2 y x

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


	
	

