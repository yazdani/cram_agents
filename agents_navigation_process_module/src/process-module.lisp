;;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
;;; All rights reserved.
;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to 
;;;       endorse or promote products derived from this software without 
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :agents-navigation-process-module)

(defparameter *navigation-enabled* t)
(defvar *base-client* nil)
(defvar *twist-client* nil)

(defvar *min-angle* (* -135.0 (/ pi 180))
  "When the angle to the goal is greater than *MIN-ANGLE*,
  controller might be used.")
(defvar *max-angle* (* 135.0 (/ pi 180))
  "When the angle to the goal is smaller than *MIN-ANGLE*,
  controller might be used.")
(defvar *max-goal-distance* 2.0
  "When the distance to goal is smaller than *GOAL-MAX-DISTANCE*,
  we might use controller.")
(defvar *xy-goal-tolerance* 0.15)
(defvar *yaw-goal-tolerance* 0.25)

(defun init-agents-navigation-process-module ()
  (setf *base-client* (actionlib:make-action-client
                            "follower"
                            "hector_quadrotor_msgs/followerAction")))
  ;; (setf *twist-client* (actionlib:make-action-client
  ;;                      "/command/twist"
  ;;                      "geometry_msgs/TwistStamped"))

  ;; (when (roslisp:has-param "~navigation_process_module/navp_min_angle")
  ;;   (setf *navp-min-angle* (roslisp:get-param "~navigation_process_module/navp_min_angle")))
  ;; (when (roslisp:has-param "~navigation_process_module/navp_max_angle")
  ;;   (setf *navp-max-angle* (roslisp:get-param "~navigation_process_module/navp_max_angle")))
  ;; (when (roslisp:has-param "~navigation_process_module/navp_max_goal_distance")
  ;;   (setf *navp-max-goal-distance* (roslisp:get-param "~navigation_process_module/navp_max_goal_distance")))
  ;; (when (roslisp:has-param "~navigation_process_module/xy_goal_tolerance")
  ;;   (setf *xy-goal-tolerance* (roslisp:get-param "~navigation_process_module/xy_goal_tolerance")))
  ;; (when (roslisp:has-param "~navigation_process_module/yaw_goal_tolerance")
  ;;   (setf *yaw-goal-tolerance* (roslisp:get-param "~navigation_process_module/yaw_goal_tolerance"))))

(roslisp-utilities:register-ros-init-function
 init-agents-navigation-process-module)

(defun make-action-goal (pose)
  (roslisp:make-message
   "geometry_msgs/Pose"
   position 
   (roslisp:make-message "geometry_msgs/Point" 
                         :x (cl-transforms:x (cl-transforms:origin pose))
                         :y  (cl-transforms:y (cl-transforms:origin pose))
                         :z   (cl-transforms:z (cl-transforms:origin pose)))
   orientation
   (roslisp:make-message "geometry_msgs/Quaternion" 
                         :x (cl-transforms:x (cl-transforms:orientation pose))
                         :y  (cl-transforms:y (cl-transforms:orientation pose))
                         :z   (cl-transforms:z (cl-transforms:orientation pose))
                         :z   (cl-transforms:w (cl-transforms:orientation pose)))))
   
;; (defun use-twist? (goal-pose)
;;   (let* ((pose-in-base (tf:transform-pose
;;                         *tf* :pose goal-pose
;;                         :target-frame "/base_link"))
;;          (goal-dist (cl-transforms:v-norm
;;                      (cl-transforms:origin pose-in-base)))
;;          (goal-angle (atan
;;                       (cl-transforms:y
;;                        (cl-transforms:origin pose-in-base))
;;                       (cl-transforms:x
;;                        (cl-transforms:origin pose-in-base)))))
;;     (and (< goal-dist 2)
;;          (> goal-angle 2)
;;          (< goal-angle 2))))

 (defun goal-reached? (goal-pose)
(format t "reached goal-pose: ~a~%" goal-pose)
   (let* ((pose-in-base (tf:transform-pose
                         *tf* :pose goal-pose
                         :target-frame "/base_stabilized"))
;; lookup-transform
;; 			 cram-roslisp-common:*tf*
;; 			 :time 0.0 :source-frame
;; 			 "/base_stabilized"
;; 			 :target-frame
;; 			 "map"))
	  (goal-dist (cl-transforms:v-norm 
		      (cl-transforms:origin 
		       pose-in-base))))
;;                      (cl-transforms:origin pose-in-base)))
;;          (goal-angle (second
;;                       (multiple-value-list
;;                           (cl-transforms:quaternion->axis-angle
;;                            (cl-transforms:orientation pose-in-base))))))
    (cond ((and (> goal-dist 2))
;;                 (> (abs goal-angle) 2))
           (roslisp:ros-warn
            (agents-nav process-module)
            "Goal not reached. Linear distance: ~a~%"
            goal-dist)
           nil)
          (t t))))


(defun call-nav-action (client desig)
  (format t "call-nav-action and the desig is ~a~%" desig)
  (let* ((goal-pose (reference desig))
         (goal-pose-in-fixed-frame
           (when (tf:wait-for-transform
                  *tf* :time (tf:stamp goal-pose)
                       :target-frame designators-ros:*fixed-frame*
                       :source-frame (tf:frame-id goal-pose)
                       :timeout 1.0)
             (tf:transform-pose
              *tf* :pose goal-pose
                   :target-frame designators-ros:*fixed-frame*))))
    (unless goal-pose-in-fixed-frame
      (error 'tf:tf-lookup-error :frame (tf:frame-id goal-pose)))
    (multiple-value-bind (result status)
        (actionlib:send-goal-and-wait
         client (make-action-goal goal-pose-in-fixed-frame)
         :result-timeout 1.0)
      (declare (ignore result status))
      (roslisp:ros-info (agents-nav process-module) "Nav action finished.")
      (unless (and ;; (eq status :succeeded)
               (goal-reached? (tf:copy-pose-stamped
                               goal-pose-in-fixed-frame
                               :stamp 0)))
        (cpl:fail 'location-not-reached-failure
                  :location desig)))))


(def-process-module agents-navigation-process-module (goal)
  (format t "what is goal? goal: ~a~%"goal)
  (when *navigation-enabled*
    (unwind-protect
         (progn
           (roslisp:ros-info (agents-nav process-module)
                             "Using controller.")
           (call-nav-action *base-client* (reference goal)))
         (roslisp:ros-info (agents-nav process-module) "Navigation finished.")
      (cram-plan-knowledge:on-event (make-instance 'cram-plan-knowledge:robot-state-changed))
      )))
