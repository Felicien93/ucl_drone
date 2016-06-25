
(cl:in-package :asdf)

(defsystem "ucl_drone-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetPoseRef" :depends-on ("_package_SetPoseRef"))
    (:file "_package_SetPoseRef" :depends-on ("_package"))
    (:file "MapChunk" :depends-on ("_package_MapChunk"))
    (:file "_package_MapChunk" :depends-on ("_package"))
    (:file "FeaturesArray" :depends-on ("_package_FeaturesArray"))
    (:file "_package_FeaturesArray" :depends-on ("_package"))
    (:file "Projection" :depends-on ("_package_Projection"))
    (:file "_package_Projection" :depends-on ("_package"))
  ))