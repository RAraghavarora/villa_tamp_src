(define (stream pick-place)
  (:stream sample-grasp
    :inputs (?o)
    :domain (Object ?o)
    :outputs (?g ?bq)
    :certified (and (Grasp ?o ?g) (BConf ?bq))
  )

  (:stream sample-pose
    :inputs (?o)
    :domain (Object ?o)
    :outputs (?p)
    :certified (Pose ?o ?p)
  )

  ; (:stream sample-place
  ;   :inputs (?o ?t)
  ;   :domain (and (Object ?o) (Table ?t))
  ;   :outputs (?p)
  ;   :certified (Pose ?o ?p)
  ; )
)