(in-package :agents-nav-pm)

(def-fact-group agents-navigation-designators (action-desig)
  
  (<- (action-desig ?designator ?goal-location)
      (desig-prop ?designator (type navigation))
      (desig-prop ?designator (goal ?goal-location))))

(def-fact-group navigation-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator agents-navigation-process-module)
    (desig-prop ?designator (type navigation)))

  (<- (available-process-module agents-navigation-process-module)
    (not (projection-running ?_))))
