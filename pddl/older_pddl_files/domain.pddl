(define (domain hsr-base)
  (:requirements :strips)
  (:predicates
    (AtBConf ?q)
    (BaseMotion ?q1 ?t ?q2)
    (BTraj ?t)
  )

  (:action move_base
    :parameters (?q1 ?t ?q2)
    :precondition (and (BaseMotion ?q1 ?t ?q2) 
                       (AtBConf ?q1))
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)))
  )
)