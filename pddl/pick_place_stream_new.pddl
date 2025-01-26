(define (stream pick-place)
  (:stream sample-grasp
    :inputs (?o)
    :domain (Object ?o)
    :outputs (?g ?bq)
    :certified (and (Grasp ?o ?g) (BConf ?bq))
  )

  (:stream sample-place
    :inputs (?o ?l)
    :domain (and (Object ?o) (Loc ?l))
    :outputs (?p ?bq)
    :certified (and (Supported ?o ?p ?l) 
                   (Pose ?o ?p)
                   (BConf ?bq))
  )
)