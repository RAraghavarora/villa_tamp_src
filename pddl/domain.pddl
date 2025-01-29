(define (domain pick-place)
 (:requirements :strips)
  (:predicates
    (Arm ?a)
    (Item ?o)
    (Uncertain ?o)
    (Surf ?s)
    (HandEmpty ?a)
    (Conf ?q)
    (AtConf ?q)
    (Detected ?q)
    (At ?o ?s)
    (Holding ?a ?o)
  )

   (:functions
    (PickCost)
    (PlaceCost)
    (DetectCost ?o ?s))

  (:action pick
    :parameters (?a ?o ?s ?q)
    :precondition (and (Arm ?a)
                       (HandEmpty ?a)
                       (Item ?o)
                       (Surf ?s)
                       (At ?o ?s)
                       (Conf ?q)
                       (At ?q ?s)
                       (not (Uncertain ?o))
                       (AtConf ?q))
    :effect (and (Holding ?a ?o)
                 (not (At ?o ?s))
                 (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost))))
  
  (:action place
    :parameters (?a ?o ?s ?q1 ?q2)
    :precondition (and (Arm ?a)
                       (Item ?o)
                       (Surf ?s)
                       (Conf ?q1)
                       (Conf ?q2)
                       (At ?q2 ?s)
                       (not (HandEmpty ?a))
                       (AtConf ?q1))
    :effect (and (At ?o ?s)
                 (HandEmpty ?a)
                 (AtConf ?q2)
                 (not (AtConf ?q1))
                 (increase (total-cost) (PlaceCost))))

  (:action detect
    :parameters (?o ?s ?q1 ?q2)
    :precondition (and (Item ?o) 
                       (Surf ?s)
                       (Conf ?q1)
                       (Conf ?q2)
                       (AtConf ?q1)
                       (At ?q2 ?s)
                       (Uncertain ?o)
                       (not (Detected ?q2)))
    :effect (and (At ?o ?s)
                 (AtConf ?q2)
                 (Detected ?q2)
                 (not (Uncertain ?o))
                 (not (AtConf ?q1))
                 (increase (total-cost) (DetectCost ?o ?s))))

)