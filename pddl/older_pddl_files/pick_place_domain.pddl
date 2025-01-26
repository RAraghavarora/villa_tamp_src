(define (domain pick-place)
  (:requirements :strips)
  (:predicates
    (Arm ?a)                  ; Represents the robot arm
    (ArmConf ?q)              ; Arm configuration
    (AtArmConf ?a ?q)         ; Current arm configuration
    (ArmMotion ?a ?q1 ?t ?q2) ; Motion between configurations
    (ATraj ?t)                ; Arm trajectory
    (Object ?o)               ; An object that can be manipulated
    (Grasp ?o ?g)             ; A grasp configuration for an object
    (AtPose ?o ?p)            ; Object's current pose
    (Pose ?o ?p)              ; Valid pose for an object
    (Graspable ?o)            ; Object can be grasped
    (HandEmpty)               ; Gripper is empty
    (Holding ?o)              ; Robot is holding an object
    (Kin ?a ?o ?p ?g ?q ?bq)  ; Inverse kinematics solution
    (BConf ?q)
    (AtBConf ?q)
    (BaseMotion ?q1 ?t ?q2)
    (BTraj ?t)                ; base trajectory
  )

  (:action move_arm
    :parameters (?a ?q1 ?t ?q2)
    :precondition (and (ArmMotion ?a ?q1 ?t ?q2)
                       (AtArmConf ?a ?q1))
    :effect (and (AtArmConf ?a ?q2)
                 (not (AtArmConf ?a ?q1)))
  )

  (:action pick
    :parameters (?a ?o ?p ?g ?q ?bq)
    :precondition (and (Arm ?a)
                       (Object ?o)
                       (Pose ?o ?p)
                       (Grasp ?o ?g)
                       (AtPose ?o ?p)
                       (Graspable ?o)
                       (HandEmpty)
                       (Kin ?a ?o ?p ?g ?q ?bq)
                       (BConf ?bq)
                       (AtArmConf ?a ?q)
                       (AtBConf ?bq))
    :effect (and (Holding ?o)
                 (not (AtPose ?o ?p))
                 (not (HandEmpty)))
  )

  (:action move_base
    :parameters (?q1 ?t ?q2)
    :precondition (and (BaseMotion ?q1 ?t ?q2) 
                        (BConf ?q1)
                        (BConf ?q2)
                       (AtBConf ?q1))
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)))
  )
)