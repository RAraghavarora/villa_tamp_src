(define (stream hsr-tamp)
  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    :outputs (?t)
    :certified (and (BTraj ?t) (BaseMotion ?q1 ?t ?q2))
  )
)