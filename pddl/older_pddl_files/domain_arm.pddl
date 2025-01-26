(define (domain hsr-arm)
  (:requirements :strips)
  (:predicates
    (Arm ?a)                  ; Represents the robot arm
    (ArmConf ?q)              ; Arm configuration
    (AtArmConf ?a ?q)         ; Current arm configuration
    (ArmMotion ?a ?q1 ?t ?q2) ; Motion between configurations
    (ATraj ?t)                ; Arm trajectory
  )

  (:action move_arm
    :parameters (?a ?q1 ?t ?q2)
    :precondition (and (ArmMotion ?a ?q1 ?t ?q2)
                       (AtArmConf ?a ?q1))
    :effect (and (AtArmConf ?a ?q2)
                 (not (AtArmConf ?a ?q1)))
  )
)