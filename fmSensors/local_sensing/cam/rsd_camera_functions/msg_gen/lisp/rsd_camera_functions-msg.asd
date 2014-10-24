
(cl:in-package :asdf)

(defsystem "rsd_camera_functions-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "line_points" :depends-on ("_package_line_points"))
    (:file "_package_line_points" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
  ))