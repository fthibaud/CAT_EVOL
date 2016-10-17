(* Float coords 2D --------------------------------------------------------- *)

type t = float * float

val mul: float -> t -> t
val add: t -> t -> t
val sub: t -> t -> t

val sca: t -> t -> float
val det: t -> t -> float

val norm2: t -> float
val norm: t -> float

val bary: float * t -> float * t -> float -> t

val pi: float
val twopi: float
val radians: float
val degrees: float
val angle: t -> float
val polar: float -> t
val mod_twopi: float -> float
val add_angle: float -> float -> float
val sub_angle: float -> float -> float

val dist_seg_seg: t * t -> t * t -> float
