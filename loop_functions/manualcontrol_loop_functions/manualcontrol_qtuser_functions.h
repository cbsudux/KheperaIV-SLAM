
#ifndef MANUALCONTROL_QTUSER_FUNCTIONS_H
#define MANUALCONTROL_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <controllers/offline_slam/offline_slam.h>

using namespace argos;

class CManualControlQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CManualControlQTUserFunctions();

   virtual ~CManualControlQTUserFunctions() {}

   /**
    * Called when a key press event occurs.
    * The focus must be on the QTOpenGL widget.
    * QTOpenGL reserves the following keys for camera movement:
    * - arrows
    * - Q W E A S D
    * If this function does not manage a keypress, it must end by
    * calling CQTOpenGLWidget::KeyPressed().
    */
   virtual void KeyPressed(QKeyEvent* pc_event);

   /**
    * Called when a key release event occurs.
    * The focus must be on the QTOpenGL widget.
    * QTOpenGL reserves the following keys for camera movement:
    * - arrows
    * - Q W E A S D
    * If this function does not manage a key release, it must end by
    * calling CQTOpenGLWidget::KeyReleased().
    */
   virtual void KeyReleased(QKeyEvent* pc_event);

   /**
    * Called every time an entity is selected.
    * @param c_entity The selected entity.
    */
   virtual void EntitySelected(CEntity& c_entity);

   /**
    * Called every time an entity is deselected.
    * @param c_entity The deselected entity.
    */
   virtual void EntityDeselected(CEntity& c_entity);

private:

   /**
    * Sets the robot direction from a key event.
    */
   void SetDirectionFromKeyEvent();

private:

   /**
    * Robot direction.
    */
   enum EDirection {
      DIRECTION_FORWARD = 0,
      DIRECTION_BACKWARD,
      DIRECTION_LEFT,
      DIRECTION_RIGHT
   };

   /**
    * Pointer to the controller of the currently selected entity.
    * NULL if no robot is selected.
    */
   CKheperaIVManualControl* m_pcController;

   /**
    * Current state of each key.
    */
   UInt8 m_punPressedKeys[4];

};

#endif
