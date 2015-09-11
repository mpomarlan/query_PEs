;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem query-pes
  :name "query-pes"
  :author "Mihai Pomarlan <mpomarlan@yahoo.co.uk>"
  :version "0.1"
  :maintainer "Mihai Pomarlan <mpomarlan@yahoo.co.uk>"
  :licence "BSD"
  :description "Query PE server and generate CRAM plans"
  :long-description "."

  :depends-on (trivial-garbage
               alexandria
               roslisp
               actionlib
               cram-language
               cram-reasoning
               designators
               cram-language-designator-support
               process-modules
               planning_msgs-srv)
  :components

  ((:module "src"
    :components
    ((:file "package")
     (:file "read-plans" :depends-on ("package"))))))

