(in-package :cl-user)

(desig-props:def-desig-package agents-navigation-process-module
  (:nicknames :agents-nav-pm)
  (:use #:common-lisp
        #:cram-reasoning
        #:cram-designators
        #:cram-process-modules
        #:cram-roslisp-common
        #:cram-plan-failures)
  (:export #:agents-navigation-process-module #:*navigation-enabled*)
  (:desig-properties #:type #:navigation #:goal #:to))
