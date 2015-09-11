;;;
;;; Copyright (c) 2015, Mihai Pomarlan <mpomarlan@yahoo.co.uk>,
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :qpe)

(defun get-type-sym (type-str)
  (cond
    ((equal (string-upcase type-str) "CRAM_FUNCTION")
      'achieve)
    ((equal (string-upcase type-str) "ACHIEVE")
      'achieve)
    ((equal (string-upcase type-str) "PERCIEVE")
      'percieve)
    ((equal (string-upcase type-str) "NAVIGATE")
      'navigate)
    (T
      (error "Unrecognized type-str. Should be one of achieve perceive navigate."))))

(defun get-step (type-str par-str bdgs-str)
  (let* ((type-sym (get-type-sym type-str))
         (par-syms (read-from-string par-str))
         (bdgs (read-from-string bdgs-str)))
    (cons type-sym (cut:substitute-vars par-syms bdgs))))

(defun msg->bindings (bindings-msg)
  (let* ((retq "(")
         (bindings-msg (coerce bindings-msg 'list)))
    (loop for binding in bindings-msg do
      (roslisp:with-fields (key value) binding
        (setf retq (concatenate 'string retq "(" key " . " value ")"))))
    (concatenate 'string retq ")")))

(defun msg->call-pattern (call-pattern-msg)
  (let* ((retq "(")
         (call-pattern-msg (coerce call-pattern-msg 'list)))
    (loop for par in call-pattern-msg do
      (setf retq (concatenate 'string retq " " par " ")))
    (concatenate 'string retq ")")))

(defun bindings->msg (bindings)
  (coerce
    (mapcar (lambda (binding)
              (roslisp:make-msg "planning_msgs/Binding"
                                :type 3
                                :key (string-downcase (format nil "~a" (car binding)))
                                :value (string-downcase (format nil "~a" (cdr binding)))))
            bindings)
    'vector))

(defun build-body (steps)
  (mapcar (lambda (step)
            (roslisp:with-fields ((type-str type) pattern bindings) step
              (get-step type-str (concatenate 'string "(" pattern ")") (msg->bindings bindings))))
          (coerce steps 'list)))

(defun get-bound-call-pattern (pattern call-pattern bindings)
  (cons
    pattern
    (mapcar (lambda (par)
              (cut:var-value par bindings)) 
            call-pattern)))

(defun get-recipes (goal pattern bindings)
  (let* ((nilary nil)
         (atomics nil)
         (pattern-string (format nil "~a" pattern))
         (pattern-string (subseq pattern-string 1 (- (length pattern-string) 1)))
         ;;(pattern-string (concatenate 'string (fmp/name goal) " " pattern-string))
         (pattern-string (string-downcase pattern-string))
         (result (roslisp:call-service "planning_server/plan"
                        "planning_msgs/Planning"
                        :pattern pattern-string
                        :bindings (bindings->msg bindings))))
    (roslisp:with-fields (plans) result
      (let* ((plans (coerce plans 'list)))
        (loop for plan in plans do
              ;;for j from 0 upto 0 do
          ;;(format t "PLAN-MSG:~a~%" plan)
          (roslisp:with-fields (score steps) plan
            (let* ((steps-len (length steps))
                   (bodies (make-array steps-len :initial-element nil))
                   (patterns (make-array steps-len :initial-element nil))
                   (ids (make-array steps-len :initial-element nil))
                   (steps (coerce steps 'list)));;(body (build-body steps)))
              (loop for step in steps
                    for k from 0 upto (- steps-len 1) do
                (roslisp:with-fields (id) step
                  (setf (elt ids k) id)))
              (loop for step in steps 
                    for k from 0 upto (- steps-len 1) do
                (roslisp:with-fields (parent pattern call_pattern bindings) step
                  (let* ((bindings (read-from-string (msg->bindings bindings)))
                         (call-pattern (read-from-string (msg->call-pattern call_pattern)))
                         (pattern (read-from-string pattern))
                         (parent-k (search (vector parent) ids))
                         (bound-call-pattern (get-bound-call-pattern pattern call-pattern bindings)))
                    (if (equal (string-upcase (format nil "~a" (car bound-call-pattern))) 
                               (string-upcase (format nil "~a" 'with-failure-handling)))
                      (let* ((parent-pattern (if parent-k
                                               (elt patterns parent-k)
                                               nil)))
                        (setf bound-call-pattern (append bound-call-pattern parent-pattern))
                        ))
                    (if (equal (length bound-call-pattern) 1)
                      (setf nilary (cons (car bound-call-pattern) nilary)))
                    (setf (elt patterns k) bound-call-pattern)
                    (if (and (< -1 parent) parent-k)
                      (setf (elt bodies parent-k)
                            (cons (cons 'achieve bound-call-pattern)
                                  (elt bodies parent-k))))
                    ;;(format t "~a~%" bound-call-pattern)
                    )))
              ;;(format t "~a~%" `(ptr-add-goal-recipe (,goal ,pattern ,(format nil "~a" (gensym)) ,score) ,@body))
              (setf bodies (mapcar #'reverse (coerce bodies 'list)))
              (setf patterns (coerce patterns 'list))
              (setf ids (coerce ids 'list))
              (loop for body in bodies
                    for id in ids
                    for pattern in patterns do
                (progn
                  (if (not body)
                    (if (not (find pattern atomics :test #'equal))
                      (setf atomics (cons pattern atomics)))
                    (format t "~a~%" `(ptr-add-goal-recipe (,goal ,pattern ,(format nil "~a" (list pattern body)) ,score) ,@body))
                    )
                  ))
              ;;(format t "PATS: ~a~%" patterns)
              ;;(format t "BDS: ~a~%" bodies)
              ;;(format t "ATS: ~a~%" atomics)
              )))))
    (values atomics nilary)))
