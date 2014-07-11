(defsystem agents-navigation-process-module
    :author "Fereshta Yazdani"
    :depends-on (actionlib
		 process-modules
		 designators
		 roslisp-utilities
		 cram-reasoning
		 cram-roslisp-common
		 cram-plan-failures
		 cram-plan-knowledge
		 move_base_msgs-msg)
    :components
    ((:module "src"
	      :components
	      ((:file "package")
	       (:file "process-module" :depends-on ("package"))
	       (:file "designators" :depends-on ("package"))))))