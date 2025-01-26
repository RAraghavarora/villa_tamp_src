(define (stream hsr-arm)

  (:stream plan-arm-motion
    :inputs (?a ?q1 ?q2)
    :domain (and (Arm ?a) 
                 (ArmConf ?q1) 
                 (ArmConf ?q2))
    :outputs (?t)
    :certified (and (ATraj ?t) 
                    (ArmMotion ?a ?q1 ?t ?q2))
  )

  ; (:stream sample-grasp
  ;   :inputs (?o)
  ;   :domain (Object ?o)
  ;   :outputs (?g)
  ;   :certified (Grasp ?o ?g)
  ; )

  ; (:stream inverse-kinematics
  ;   :inputs (?a ?o ?p ?g)
  ;   :domain (and (Arm ?a) (Pose ?o ?p) (Grasp ?o ?g))
  ;   :outputs (?q)
  ;   :certified (and (ArmConf ?q) 
  ;                   (Kin ?a ?o ?p ?g ?q))
  ; )

  ; (:stream plan-arm-motion
  ;   :inputs (?a ?q1 ?q2)
  ;   :domain (and (Arm ?a) (ArmConf ?q1) (ArmConf ?q2))
  ;   :outputs (?t)
  ;   :certified (ArmMotion ?a ?q1 ?t ?q2)
  ; )
)