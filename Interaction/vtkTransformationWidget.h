//////////////////////////////////////////////////////////////////////////////
//
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VTKTRANSFORMATIONWIDGETWIDGET_H__
#define __VTKTRANSFORMATIONWIDGETWIDGET_H__

// Export macro for this module
#include "vtkUtilsModule.h"

// VTK
#include <vtk3DWidget.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkCallbackCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkLineSource.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkSphereSource.h>
#include <vtkFeatureEdges.h>

#include <vtkArcSource.h>
#include <vtkRenderer.h>

// STL
#include <array>

using RGBColorValue = std::array<double,3>;

// Forwards
class vtkTransform;
class vtkMatrix4x4;
class vtkAssembly;
class vtkActor;
class vtkCallbackCommand;
class vtkActor;
class vtkArcSource;
class vtkLineSource;
class vtkPoints;
class vtkCellArray;
class vtkPolyData;
class vtkPolyDataMapper;

#define TransformationWidgetTransformModifiedEvent (vtkCommand::UserEvent + 1)

static constexpr double YELLOW_HOVER_COLOR[3] = { 1.0, 1.0, 0.0 };

class vtkRotationArcOverlay
{
public:
  vtkRotationArcOverlay() = default;

  void AttachToRenderer(vtkRenderer* ren);
  void DetachFromRenderer();

  // Styling
  void SetColor(double r, double g, double b);
  void SetLineWidth(double w);
  void SetArcSegments(int n);        // e.g. 64
  void SetDepthOffsetLocal(double d); // small offset along axis (local units)

  // Lifecycle
  void Begin(const double axisUnit[3],
			   const double startDirUnit[3],
			   const double centerWorld[3],
			   double radiusWorld);
  void Update(const double endDirLocalUnit[3]);
  void End();

  bool IsActive() const { return this->Active; }
  
  void GetDeltaTransform(vtkTransform* t);
 

private:

  vtkSmartPointer<vtkPoints>        FillPoints;
  vtkSmartPointer<vtkCellArray>     FillPolys;
  vtkSmartPointer<vtkPolyData>      FillPolyData;
  vtkSmartPointer<vtkTriangleFilter> FillTri;
  vtkSmartPointer<vtkPolyDataMapper> FillMapper;
  vtkSmartPointer<vtkActor>          FillActor;
  double FillOpacity{0.35};

  void BuildActorsIfNeeded();
  void UpdateGeometry(const double endDirLocalUnit[3]);

  static double Clamp(double v, double lo, double hi);
  static void Normalize3(double v[3]);
  static void Cross3(const double a[3], const double b[3], double out[3]);
  static double Dot3(const double a[3], const double b[3]);
  static void RotateRodrigues(const double v[3], const double axisUnit[3], double angleRad, double out[3]);

  bool Active{false};
  bool Attached{false};

  double Center[3]   = { 0.0, 0.0, 0.0 };
  double Axis[3]{0.0, 0.0, 1.0};     
  double StartDir[3]{1.0, 0.0, 0.0}; 
  double Radius{1.0};

  double Color[3]{1.0, 1.0, 1.0};
  double LineWidth{3.0};
  int ArcSegments{64};
  double DepthOffset{0.002};
  
  vtkSmartPointer<vtkLineSource> StartLineSource;
  vtkSmartPointer<vtkLineSource> EndLineSource;
  vtkSmartPointer<vtkActor>      StartLineActor;
  vtkSmartPointer<vtkActor>      EndLineActor;

  vtkSmartPointer<vtkArcSource>  ArcSource;
  vtkSmartPointer<vtkActor>      ArcActor;

  vtkRenderer* AttachedRenderer = nullptr;
  
};


class VTKUTILS_EXPORT  vtkTransformationWidget : public vtk3DWidget
{
public:
    static vtkTransformationWidget* New();
    vtkTypeMacro(vtkTransformationWidget, vtk3DWidget);

    vtkTransformationWidget(const vtkTransformationWidget&) = delete;
    void operator=(const vtkTransformationWidget&) = delete;

    void PrintSelf(ostream& os, vtkIndent indent) override;

    // Enable/disable this widget
    void SetEnabled(int enabling) override;

    void ApplyCallbacks();

    // Various ways to place the widget
    void PlaceWidgetToPoint(double x, double y, double z);
    void PlaceWidget() override;
    void PlaceWidget(double bounds[6]) override;

    // Placing the widget at a specific 3D point
    void setVisible(bool visible, bool helper = true);

    /// Rotates around the Y axis.
    void RotateY(const float angle);

    /// Activates or deactivates rot Y mode.
    void SetRotationModeY();

    /// Activates or deactivates translation mode.
    void SetTranslationMode();

    /// Activates or deactivates the X-axis arrow handle and its helper line.
    void SetArrowXHandleActive(bool active);

    /// Activates or deactivates the Y-axis arrow handle and its helper line.
    void SetArrowYHandleActive(bool active);

    /// Activates or deactivates the Z-axis arrow handle and its helper line.
    void SetArrowZHandleActive(bool active);

    /// Activates or deactivates the rotation handle for the X-axis and its helper arc.
    void SetRotXHandleActive(bool active);

    /// Activates or deactivates the rotation handle for the Y-axis and its helper arc.
    void SetRotYHandleActive(bool active);

    /// Activates or deactivates the rotation handle for the Z-axis and its helper arc.
    void SetRotZHandleActive(bool active);

    /// Activates or deactivates the XY quad handle used for planar translation.
    void SetQuadXYHandleActive(bool active);

    /// Activates or deactivates the XZ quad handle used for planar translation.
    void SetQuadXZHandleActive(bool active);

    /// Activates or deactivates the ZY quad handle used for planar translation.
    void SetQuadZYHandleActive(bool active);

    /// Activates or deactivates the center handle (sphere) of the widget.
    void SetCenterHandleActive(bool active);

    /// Activates the permanent visibility of the Y disc.
    void SetYDiscVisible(bool active);
	
	void SetZDiscVisible(bool active);

     /// Writes this widget's pose into 't'. 
    void GetTransform(vtkTransform* t) const;

    /// Reads pose from 't' (R|t) and applies it to the widget. 
    void SetTransform(vtkTransform* t);

     /// Copies the current center into 'out[3]'. 
    void GetCenter(double center[3]) const;
	void GetCenter(double& cx, double& cy, double& cz) const;

     /// Force an interactor/render refresh (safe no-op if missing). 
    void RefreshInteractor();
	
	void SetFrontRenderer(vtkRenderer* renderer);
	void SetSceneRenderer(vtkRenderer* renderer);
	
	void GetDeltaTransform(vtkTransform* t);
	void SetRotationModeZ();
	void GetInteractionStartCenter(double center[3]) const;

private:

    void UnregisterAllEvents();
	
	void AttachRotationOverlayToRenderer(vtkRenderer* ren);
	void DetachRotationOverlayFromRenderer(vtkRenderer* ren);
	void AttachTranslationOverlayToRenderer(vtkRenderer* ren);
	void DetachTranslationOverlayFromRenderer(vtkRenderer* ren);
	
	vtkRenderer* m_frontRenderer;
	vtkRenderer* m_sceneRenderer;

    unsigned long LeftButtonPressCallbackId  = 0;
    unsigned long LeftButtonReleaseCallbackId = 0;
    unsigned long MouseMoveCallbackId        = 0;
    unsigned long CameraModifiedCallbackId   = 0;

    double LastOrigin[3] = { 0.0, 0.0, 0.0 };
	
	void CaptureRotationStartDirWorld();

    /// WidgetCoordinateSystem holds the origin and axis directions for the widget.
    struct WidgetCoordinateSystem
    {
        std::array<double, 3> origin;
        std::array<double, 3> dirX;
        std::array<double, 3> dirY;
        std::array<double, 3> dirZ;

        /// Constructor initializes standard right-handed coordinate system.
        WidgetCoordinateSystem()
            : origin({ 0.0, 0.0, 0.0 }),
              dirX({ 1.0, 0.0, 0.0 }),
              dirY({ 0.0, 1.0, 0.0 }),
              dirZ({ 0.0, 0.0, 1.0 })
        {}

        void SetOrigin(double x, double y, double z) { origin = { x, y, z }; }
        void SetDirX(double x, double y, double z)   { dirX   = { x, y, z }; }
        void SetDirY(double x, double y, double z)   { dirY   = { x, y, z }; }
        void SetDirZ(double x, double y, double z)   { dirZ   = { x, y, z }; }

        void GetOrigin(double* p) const { for (int i = 0; i < 3; ++i) p[i] = origin[i]; }
        void GetX(double* p) const      { for (int i = 0; i < 3; ++i) p[i] = dirX[i]; }
        void GetY(double* p) const      { for (int i = 0; i < 3; ++i) p[i] = dirY[i]; }
        void GetZ(double* p) const      { for (int i = 0; i < 3; ++i) p[i] = dirZ[i]; }

        vtkSmartPointer<vtkTransform> AsTransform() const
        {
            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->Identity();

            vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();

            // First column: dirX
            matrix->SetElement(0, 0, dirX[0]);
            matrix->SetElement(1, 0, dirX[1]);
            matrix->SetElement(2, 0, dirX[2]);

            // Second column: dirY
            matrix->SetElement(0, 1, dirY[0]);
            matrix->SetElement(1, 1, dirY[1]);
            matrix->SetElement(2, 1, dirY[2]);

            // Third column: dirZ
            matrix->SetElement(0, 2, dirZ[0]);
            matrix->SetElement(1, 2, dirZ[1]);
            matrix->SetElement(2, 2, dirZ[2]);

            // Fourth column: origin (translation)
            matrix->SetElement(0, 3, origin[0]);
            matrix->SetElement(1, 3, origin[1]);
            matrix->SetElement(2, 3, origin[2]);

            // Bottom row
            matrix->SetElement(3, 0, 0.0);
            matrix->SetElement(3, 1, 0.0);
            matrix->SetElement(3, 2, 0.0);
            matrix->SetElement(3, 3, 1.0);

            transform->SetMatrix(matrix);
            return transform;
        }
    };
	
	
	  // -----------------------------
	  // Compass / gizmo parameters
	  // -----------------------------

	  // Overall scale / pixel sizing
	  double CompassDesiredPixelSizePx = 250.0;   
	  double CompassAssemblyScale      = 0.7;
	  
	  double RotAxisLocal[3]     = { 0.0, 0.0, 1.0 };   
	  double RotStartDirLocal[3] = { 1.0, 0.0, 0.0 };   
	  double RotStartDirWorld[3] = { 1.0, 0.0, 0.0 };

	  // Arrows
	  double CompassArrowOffset      = 0.1;
	  double CompassArrowTipLength   = 0.15;
	  double CompassArrowTipRadius   = 0.015;
	  double CompassArrowShaftRadius = 0.01;
	  int    CompassArrowTipRes      = 32;
	  int    CompassArrowShaftRes    = 32;
	  double CompassArrowScale       = 0.3;      

	  bool AxisLineXVisible = true;
	  bool AxisLineYVisible = true;
	  bool AxisLineZVisible = true;	  
	  void SetAxisLineXVisible(bool visible);
	  void SetAxisLineYVisible(bool visible);
	  void SetAxisLineZVisible(bool visible);

	  // Arrow orientations
	  double CompassYArrowRotateZDeg = 90.0;
	  double CompassZArrowRotateYDeg = -90.0;

	  // Center sphere
	  double CompassCenterSphereRadius   = 0.02;
	  int    CompassSpherePhiResolution  = 30;
	  int    CompassSphereThetaResolution= 30;

	  // Helper lines 
	  double CompassHelperLineWidth      = 1.0;  
	  double CompassMaxLineExtent        = 100000.0;   
	  double CompassYModeLineHalfLength  = 0.4;   
	  double CompassLineFarClipMultiplier= 2.0;  

	  // Quads 
	  double CompassQuadMin = 0.1;
	  double CompassQuadMax = 0.2;

	  // Rotation rings / tubes
	  double CompassRotHandleRadius  = 0.43;
	  int    CompassRotPolygonSides  = 100;
	  double CompassRotTubeRadius    = 0.005;
	  int    CompassRotTubeSides     = 12;

	  // Discs 
	  int    CompassDiscSlices       = 100;
	  double CompassDiscOpacity      = 0.5;
	  double CompassDiscRotateDeg    = 90.0;

	  // Shading 
	  double CompassZArrowSpecular       = 0.6;
	  double CompassZArrowSpecularPower  = 50.0;

	  double HelperLineWidth = 2.0;

    //===========================================================================
    // Colors
    //===========================================================================
    const RGBColorValue WHITE_COLOR   = { 1.0, 1.0, 1.0 }; // White

    const RGBColorValue X_ARROW_COLOR = { 0.859, 0.353, 0.365}; // Red
    const RGBColorValue Y_ARROW_COLOR = { 0.584, 0.831, 0.247 }; // Green
    const RGBColorValue Z_ARROW_COLOR = { 0.290, 0.600, 0.973 }; // Blue

    // Hover colors
	const RGBColorValue HOVER_COLOR = { 1.0, 1.0, 0.0 }; // Yellow
    const RGBColorValue X_HOVER_COLOR = { 1.0, 0.5, 0.0 }; // Orange
    const RGBColorValue Y_HOVER_COLOR = { 0.5, 1.0, 0.0 }; // Light Green
    const RGBColorValue Z_HOVER_COLOR = { 0.0, 0.5, 1.0 }; // Light Blue
	
	const RGBColorValue X_INTERACT_COLOR = X_HOVER_COLOR; // Orange
	const RGBColorValue Y_INTERACT_COLOR = Y_HOVER_COLOR; // Light Green
	const RGBColorValue Z_INTERACT_COLOR = Z_HOVER_COLOR; // Light Blue
	const RGBColorValue CENTER_INTERACT_COLOR = Z_HOVER_COLOR;
	
	void ApplyInteractionColorsForPickedHandle();

    //===========================================================================
    // Constructor / Destructor
    //===========================================================================
    vtkTransformationWidget();
    ~vtkTransformationWidget() override;

    //===========================================================================
    // Widget Component Creation Methods
    //===========================================================================	
	bool WorldToWidgetLocal(const double pWorld[3], double pLocal[3]) const;
	void RemoveFromAllRenderers();

    /// Create the default visual representation of the widget.
    void CreateDefaultRepresentation();

    // --- Dimension Lines ---
    void CreateZLine(double MaxLineExtent);
    void CreateYLine(double MaxLineExtent);
    void CreateXLine(double MaxLineExtent);

    // --- Arrow Handles ---
    void CreateZArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset);
    void CreateYArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset);
    void CreateXArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset);

    // --- Quad Handles (for planar translation) ---
    void CreateXYQuad();
    void CreateXZQuad();
    void CreateZYQuad();

    // --- Rotation Handles ---
    void CreateRotXHandle(double radius);
    void CreateRotYHandle(double radius);
    void CreateRotZHandle(double radius);

    // --- Arc Discs ---
    vtkSmartPointer<vtkActor> CreateDisc(double radius, int plane, const RGBColorValue color, double rotationAngle, const double rotationAxis[3]);
    void CreateXDisc(double radius);
    void CreateYDisc(double radius);
    void CreateZDisc(double radius);

    /// Update the positions of the dimension lines.
    void UpdateLinePositions();

    /// Set the color for a given handle actor.
    void SetHandleColor(vtkSmartPointer<vtkActor> actor, const RGBColorValue color);

    /// Create a sphere actor at the widget’s center.
    void CreateCenterSphere(double radius);

    /// Set widget helpers visible/invisible.
    void SetArcsAndLinesVisibility(bool arcX, bool arcY, bool arcZ, bool lineX, bool lineY, bool lineZ);

    /// Set widget handles visible/invisible.
    void SetHandlesVisibility(bool arrowX, bool arrowY, bool arrowZ,
                              bool quadXY, bool quadXZ, bool quadZY,
                              bool rotX,  bool rotY,  bool rotZ,
                              bool center);

    /// Redraw the widget.
    void UpdateWidgetGeometry();

    /// Send the event.
    void PropagateTransformModifiedEvent();

    /// Computes a scale factor.
    double ComputeScalingFactor(double factor);

    // Handle Activation flags
    bool CenterActive = true;
    bool ArrowXActive = true;
    bool ArrowYActive = true;
    bool ArrowZActive = true;
    bool RotXActive   = true;
    bool RotYActive   = true;
    bool RotZActive   = true;
    bool QuadXYActive = true;
    bool QuadXZActive = true;
    bool QuadZYActive = true;
    bool YDiscVisible = false;
	bool ZDiscVisible = false;

    //===========================================================================
    // Widget Visualization
    //===========================================================================
	
	vtkSmartPointer<vtkActor> OcclusionSphereActor;
	vtkSmartPointer<vtkActor> OcclusionSphereDepthActor;
	void CreateOcclusionSphere(double radius);

    // Arrow actors for each axis.
    vtkSmartPointer<vtkTransform>  AxesTransform;
    vtkSmartPointer<vtkTransform>  ScaleTransform;
    vtkSmartPointer<vtkAssembly>   AxesAssembly;

    // Actors for the three axis arrows
    vtkSmartPointer<vtkActor> ArrowXActor;
    vtkSmartPointer<vtkActor> ArrowYActor;
    vtkSmartPointer<vtkActor> ArrowZActor;

    // Rotation handle actors.
    vtkSmartPointer<vtkActor>  RotXActor;
    vtkSmartPointer<vtkActor>  RotYActor;
    vtkSmartPointer<vtkActor>  RotZActor;
	
    // Quad actors for planar movement.
    vtkSmartPointer<vtkActor> QuadXYActor;
    vtkSmartPointer<vtkActor> QuadXZActor;
    vtkSmartPointer<vtkActor> QuadZYActor;

    // Arrow line actors.
    vtkSmartPointer<vtkActor> ArrowXLineActor;
    vtkSmartPointer<vtkActor> ArrowYLineActor;
    vtkSmartPointer<vtkActor> ArrowZLineActor;

    // Arc actors for rotation visualization.
    vtkSmartPointer<vtkActor> ArcXActor;
    vtkSmartPointer<vtkActor> ArcYActor;
    vtkSmartPointer<vtkActor> ArcZActor;

    // Center sphere actor.
    vtkSmartPointer<vtkActor> CenterSphereActor;

    // Additional arrow line actors.
    vtkSmartPointer<vtkActor> YModeLineActor;
	
	void SetOcclusionSphereVisible(bool visible);
	
	// -----------------------------------------------------------------------------
	// Rotation overlay (only visible while rotating)
	// -----------------------------------------------------------------------------
	vtkSmartPointer<vtkBillboardTextActor3D> RotationTextActor;

    bool InteractionInProgress = false;
	bool translationMode = false; 
    void SetOcclusionForInteraction(bool interacting);

	bool   RotationOverlayActive = false;
	double RotationBillboardPixelHeightPx  = 70.0;  // target height in pixels (approx)
	double RotationBillboardMinMul         = 0.75;
	double RotationBillboardMaxMul         = 1.75;
	double RotationBillboardAngleForMaxDeg = 90.0;  // reach max multiplier around this angle

	void CreateRotationOverlayIfNeeded();
	void BeginRotationOverlay(const double centerWorld[3]);
	void UpdateRotationOverlay(double thetaDeg, const double centerWorld[3], const double hitWorld[3]);
	void EndRotationOverlay();
	
	vtkRotationArcOverlay RotationArcOverlay;
	void BeginRotationArcOverlay();
	void UpdateRotationArcOverlayFromHitWorld(const double hitWorld[3]);
	void EndRotationArcOverlay();

	// -----------------------------------------------------------------------------
	// Translation overlay (only visible while translating)
	// -----------------------------------------------------------------------------
	vtkSmartPointer<vtkLineSource> TranslationLineSource;
	vtkSmartPointer<vtkActor>      TranslationLineActor;
	vtkSmartPointer<vtkBillboardTextActor3D> TranslationTextActor;
    vtkSmartPointer<vtkSphereSource> TranslationStartMarkerSource;
    vtkSmartPointer<vtkPolyDataMapper> TranslationStartMarkerMapper;
    vtkSmartPointer<vtkActor> TranslationStartMarkerActor;
	vtkSmartPointer<vtkPolyDataMapper> TranslationLineMapper;

	bool   TranslationOverlayActive = false;
	double TranslationStartWorld[3] = { 0.0, 0.0, 0.0 };

	// "Billboard" sizing behavior
	double TranslationBillboardPixelHeightPx = 70.0;  // target height in pixels (approx)
	double TranslationBillboardMinMul        = 0.75;  // min size multiplier
	double TranslationBillboardMaxMul        = 1.75;  // max size multiplier
	double TranslationBillboardDistForMax    = 0.50;  // world distance to reach max multiplier

	void CreateTranslationOverlayIfNeeded();
    void BeginTranslationOverlay(const double startWorld[3]);
    void UpdateTranslationOverlay(const double currentWorld[3]);
	void EndTranslationOverlay();

    //===========================================================================
    // Picking and Dragging Flags
    //===========================================================================

    double LastPickPosition[3];

    // Flags for axis arrow picking.
    bool arrowXPicked = false;
    bool arrowYPicked = false;
    bool arrowZPicked = false;

    // Flags for quad picking.
    bool quadXYPicked = false;
    bool quadXZPicked = false;
    bool quadZYPicked = false;

    // Flags for rotation handle picking.
    bool rotXPicked = false;
    bool rotYPicked = false;
    bool rotZPicked = false;

    // Flags indicating if a drag operation is in progress.
    bool isRotDragging   = false;
    bool isAxisDragging  = false;
    bool isPlaneDragging = false;
    bool isCenterDragging = false;
    bool centerPicked    = false;

    //===========================================================================
    // Event Handling and Callback Methods
    //===========================================================================

    /// Update the scale of the widget to match the camera view.
    void UpdateScaleToCamera(vtkObject* caller, unsigned long, void*);

    /// Handle mouse move events.
    void OnMouseMove(vtkObject* caller, unsigned long eventId, void* callData);

    /// Determine which handle (if any) was picked based on the mouse position.
    void PickHandle(int mousePos[2]);

    /// Reset the widget’s interaction state.
    void ResetState();

    /// Static callback for processing VTK events.
    static void ProcessEvents(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);

    vtkSmartPointer<vtkCallbackCommand> EventCallbackCommand;

    // --- Mouse Button Event Handlers ---
    void onLeftButtonDown();
    void onLeftButtonUp();

    double m_rotationTheta; ///< Current rotation angle used in rotation interactions.

    // --- Movement and Rotation Operations ---
    void moveAlongAxis(int x, int y);
    void movePlane(int x, int y);
    void rotateAxis(int x, int y);
    void moveCenter(int x, int y);

    void Init();

    //===========================================================================
    // Math Helper Functions
    //===========================================================================
    bool rayPlaneIntersect(const double origin[3], const double direction[3],
                           const double n[3], const double offset, double hitPoint[3]);
    void vtkDivide(const double a[3], const double b, double c[3]);
    void vtkScale(const double a[3], const double b, double c[3]);
    void vtkAdd(const double a[3], const double b[3], double c[3]);

    //===========================================================================
    // Variables for Mouse Interaction
    //===========================================================================
    int    startMouseX = 0, startMouseY = 0; ///< Starting mouse coordinates for drag operations.
    double m_pickedPointOffset[3];           ///< Offset between the pick point and widget center.

    //===========================================================================
    // Widget Coordinate Systems
    //===========================================================================

    WidgetCoordinateSystem widgetCS;      ///< Current widget coordinate system.
    WidgetCoordinateSystem widgetCSStart; ///< Baseline coordinate system at the start of an interaction.
	
	void StartInteraction();
	void EndInteraction();
	
};

#endif // __VTKTRANSFORMATIONWIDGETWIDGET_H__
