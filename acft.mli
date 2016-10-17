type time = float  (* Seconds *)
type speed = float  (* Knots *)

type pln = (time * Xy.t) array  (* Flight plan *)


type t = {  (* Aircraft *)
  speed: speed;              (* Nominal speed *)
  mutable pln: pln;          (* Real flightplan *)
  mutable leg: int;          (* Current leg in the flight plan *)
  mutable predict: pln;      (* Prediction *)
  mutable route: Xy.t array; (* Positions every delta sec. *)
}

val delta: time           (* trace time steps (sec) *)
val vector: time          (* speed vector size (sec) *)
val max_speed: speed      (* max speed (Knots) *)
val dspeed: float         (* Speed uncertainty rate *)
val sep: float            (* minimal separation (Nm) *)

(* Flight plan operations *)

val pln_to_string: pln -> string
val pln_of_string: string -> pln

val create_pln: speed -> float -> time -> Xy.t array -> pln
val create_route: pln -> Xy.t array

(* Aircraft operations *)

val t_start: t -> time
val t_cur: t -> time
val t_end: t -> time

val update: t -> time -> unit
val get_pos: t -> Xy.t
val get_vector: t -> Xy.t
val get_comet: t -> Xy.t array
val reset: t -> unit

val dev: t -> time -> Xy.t -> pln
val apply_dev: t -> unit
val delay: t -> float

(* Conflicts detection *)

val pos_detect: t array -> bool array

type seg = Xy.t * Xy.t
val detect: t -> t -> time * (seg list * seg list)

(* Random traffic situation *)

val roundabout: float -> int -> t array
