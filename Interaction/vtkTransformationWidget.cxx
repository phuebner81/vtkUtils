#include "vtkTransformationWidget.h"

// VTK core
#include <vtkObjectFactory.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>

// Geometry sources & filters
#include <vtkArrowSource.h>
#include <vtkPlaneSource.h>
#include <vtkRegularPolygonSource.h>
#include <vtkTriangleFilter.h>
#include <vtkTubeFilter.h>
#include <vtkProp3DCollection.h>
#include <vtkAlgorithmOutput.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>

// Polydata building blocks
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkArcSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkLineSource.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

// Picking & paths
#include <vtkCellPicker.h>
#include <vtkAssemblyPath.h>

#include <vtkBillboardTextActor3D.h>
#include <vtkTextProperty.h>
#include <sstream>
#include <iomanip>

// Rendering pipeline 
#include <vtk_glew.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkOpenGLActor.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLState.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>
#include <vtkViewport.h>

#include <vtkLinearTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>

// STL
#include <algorithm>  // std::min, std::max
#include <cmath>      // sin, cos, tan, acos, sqrt
#include <limits>     // std::numeric_limits

class vtkDepthOnlyActor : public vtkOpenGLActor
{
public:
    static vtkDepthOnlyActor* New();
    vtkTypeMacro(vtkDepthOnlyActor, vtkOpenGLActor);

    int RenderOpaqueGeometry(vtkViewport* vp) override
    {
        auto* ren = vtkRenderer::SafeDownCast(vp);
        auto* glRen = vtkOpenGLRenderer::SafeDownCast(ren);
        if (!glRen)
        {
            return this->Superclass::RenderOpaqueGeometry(vp);
        }

        vtkOpenGLState* s = glRen->GetState();

        // Save/restore OpenGL state
        vtkOpenGLState::ScopedglColorMask savedColorMask(s);
        vtkOpenGLState::ScopedglDepthMask savedDepthMask(s);
        vtkOpenGLState::ScopedglEnableDisable savedBlend(s, GL_BLEND);

        // Depth-only: no color writes, depth writes ON, blending OFF.
        s->vtkglColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
        s->vtkglDepthMask(GL_TRUE);
        s->vtkglDisable(GL_BLEND);

        return this->Superclass::RenderOpaqueGeometry(vp);
    }
};

void vtkRotationArcOverlay::AttachToRenderer(vtkRenderer* ren)
{
  if (!ren) { return; }
  this->BuildActorsIfNeeded();

  if (!this->Attached)
  {
	ren->AddViewProp(this->FillActor);
    ren->AddViewProp(this->StartLineActor);
    ren->AddViewProp(this->EndLineActor);
    ren->AddViewProp(this->ArcActor);
    this->Attached = true;
    this->AttachedRenderer = ren;
  }
}

void vtkRotationArcOverlay::DetachFromRenderer()
{
  if (!this->Attached || !this->AttachedRenderer) { return; }

  this->AttachedRenderer->RemoveViewProp(this->FillActor);
  this->AttachedRenderer->RemoveViewProp(this->StartLineActor);
  this->AttachedRenderer->RemoveViewProp(this->EndLineActor);
  this->AttachedRenderer->RemoveViewProp(this->ArcActor);

  this->AttachedRenderer = nullptr;
  this->Attached = false;
}

void vtkRotationArcOverlay::SetColor(double r, double g, double b)
{
  this->Color[0] = r;
  this->Color[1] = g;
  this->Color[2] = b;

  if (this->StartLineActor)
  {
    this->StartLineActor->GetProperty()->SetColor(r, g, b);
  }
  if (this->EndLineActor)
  {
    this->EndLineActor->GetProperty()->SetColor(r, g, b);
  }
  if (this->ArcActor)
  {
    this->ArcActor->GetProperty()->SetColor(r, g, b);
  }
}

void vtkRotationArcOverlay::SetLineWidth(double w)
{
  this->LineWidth = w;

  if (this->StartLineActor)
  {
    this->StartLineActor->GetProperty()->SetLineWidth(w);
  }
  if (this->EndLineActor)
  {
    this->EndLineActor->GetProperty()->SetLineWidth(w);
  }
  if (this->ArcActor)
  {
    this->ArcActor->GetProperty()->SetLineWidth(w);
  }
}

void vtkRotationArcOverlay::SetArcSegments(int n)
{
  this->ArcSegments = std::max(8, n);
}

void vtkRotationArcOverlay::SetDepthOffsetLocal(double d)
{
  this->DepthOffset = d;
}

void vtkRotationArcOverlay::Begin(
  const double axisUnit[3],
  const double startDirUnit[3],
  const double centerWorld[3],
  double radiusWorld)
{
  this->BuildActorsIfNeeded();

  this->Axis[0]=axisUnit[0]; this->Axis[1]=axisUnit[1]; this->Axis[2]=axisUnit[2];
  Normalize3(this->Axis);

  this->StartDir[0]=startDirUnit[0]; this->StartDir[1]=startDirUnit[1]; this->StartDir[2]=startDirUnit[2];
  Normalize3(this->StartDir);

  this->Center[0]=centerWorld[0]; this->Center[1]=centerWorld[1]; this->Center[2]=centerWorld[2];
  this->Radius = radiusWorld;

  this->Active = true;
  this->StartLineActor->SetVisibility(true);
  this->EndLineActor->SetVisibility(true);
  this->ArcActor->SetVisibility(true);
  this->FillActor->SetVisibility(true);

  this->UpdateGeometry(this->StartDir);
}

void vtkRotationArcOverlay::Update(const double endDirLocalUnit[3])
{
  if (!this->Active)
  {
    return;
  }

  this->UpdateGeometry(endDirLocalUnit);
}

void vtkRotationArcOverlay::End()
{
  this->Active = false;

  if (this->StartLineActor) { this->StartLineActor->SetVisibility(false); }
  if (this->EndLineActor)   { this->EndLineActor->SetVisibility(false); }
  if (this->ArcActor)       { this->ArcActor->SetVisibility(false); }
  if (this->FillActor) { this->FillActor->SetVisibility(false); }
}

void vtkRotationArcOverlay::BuildActorsIfNeeded()
{
  if (this->StartLineActor && this->EndLineActor && this->ArcActor)
  {
    return;
  }

  // --- Start line
  this->StartLineSource = vtkSmartPointer<vtkLineSource>::New();
  this->StartLineSource->SetPoint1(0.0, 0.0, 0.0);
  this->StartLineSource->SetPoint2(0.0, 0.0, 0.0);

  vtkSmartPointer<vtkPolyDataMapper> startMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  startMapper->SetInputConnection(this->StartLineSource->GetOutputPort());

  this->StartLineActor = vtkSmartPointer<vtkActor>::New();
  this->StartLineActor->SetMapper(startMapper);
  this->StartLineActor->PickableOff();
  this->StartLineActor->GetProperty()->SetColor(this->Color);
  this->StartLineActor->GetProperty()->SetLineWidth(this->LineWidth);
  this->StartLineActor->SetVisibility(false);

  // --- End line
  this->EndLineSource = vtkSmartPointer<vtkLineSource>::New();
  this->EndLineSource->SetPoint1(0.0, 0.0, 0.0);
  this->EndLineSource->SetPoint2(0.0, 0.0, 0.0);

  vtkSmartPointer<vtkPolyDataMapper> endMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  endMapper->SetInputConnection(this->EndLineSource->GetOutputPort());

  this->EndLineActor = vtkSmartPointer<vtkActor>::New();
  this->EndLineActor->SetMapper(endMapper);
  this->EndLineActor->PickableOff();
  this->EndLineActor->GetProperty()->SetColor(this->Color);
  this->EndLineActor->GetProperty()->SetLineWidth(this->LineWidth);
  this->EndLineActor->SetVisibility(false);

  this->ArcSource = vtkSmartPointer<vtkArcSource>::New();
  this->ArcSource->UseNormalAndAngleOn();
  this->ArcSource->SetResolution(this->ArcSegments);
	
  this->FillPoints   = vtkSmartPointer<vtkPoints>::New();
  this->FillPolys    = vtkSmartPointer<vtkCellArray>::New();
  this->FillPolyData = vtkSmartPointer<vtkPolyData>::New();
  this->FillPolyData->SetPoints(this->FillPoints);
  this->FillPolyData->SetPolys(this->FillPolys);

	  this->FillTri = vtkSmartPointer<vtkTriangleFilter>::New();
	  this->FillTri->SetInputData(this->FillPolyData);

	  this->FillMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	  this->FillMapper->SetInputConnection(this->FillTri->GetOutputPort());
	//  this->FillMapper->SetResolveCoincidentTopologyToPolygonOffset();
	//  this->FillMapper->SetResolveCoincidentTopologyPolygonOffsetParameters(2.0, 2.0);

	  this->FillActor = vtkSmartPointer<vtkActor>::New();
	  this->FillActor->SetMapper(this->FillMapper);
	  this->FillActor->PickableOff();
	  this->FillActor->GetProperty()->SetColor(this->Color);
	  this->FillActor->GetProperty()->SetOpacity(this->FillOpacity);

	  // Make it look solid + readable
	  this->FillActor->GetProperty()->SetAmbient(1.0);
	  this->FillActor->GetProperty()->SetDiffuse(0.0);
	  this->FillActor->GetProperty()->SetSpecular(0.0);
	  this->FillActor->GetProperty()->BackfaceCullingOff(); // don't disappear
	  this->FillActor->SetVisibility(false);


	vtkSmartPointer<vtkPolyDataMapper> arcMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	arcMapper->SetInputConnection(this->ArcSource->GetOutputPort());

	this->ArcActor = vtkSmartPointer<vtkActor>::New();
	this->ArcActor->SetMapper(arcMapper);
	this->ArcActor->PickableOff();
	this->ArcActor->GetProperty()->SetColor(this->Color);
	this->ArcActor->GetProperty()->SetLineWidth(this->LineWidth);
	this->ArcActor->SetVisibility(false);

	// keep your “no lighting” settings
	this->ArcActor->GetProperty()->SetAmbient(1.0);
	this->ArcActor->GetProperty()->SetDiffuse(0.0);
	this->ArcActor->GetProperty()->SetSpecular(0.0);
}

void vtkRotationArcOverlay::UpdateGeometry(const double endDirLocalUnitIn[3])
{
  double endDir[3] = { endDirLocalUnitIn[0], endDirLocalUnitIn[1], endDirLocalUnitIn[2] };
  Normalize3(endDir);

  // signed angle around axis
  double crossSE[3];
  Cross3(this->StartDir, endDir, crossSE);
  const double sinTerm = Dot3(this->Axis, crossSE);
  const double cosTerm = Clamp(Dot3(this->StartDir, endDir), -1.0, 1.0);
  const double angleRad = std::atan2(sinTerm, cosTerm);
  double angleDeg = vtkMath::DegreesFromRadians(angleRad);

  // world center (with tiny offset along axis if you want)
  const double off = this->DepthOffset;
  double c[3] = {
    this->Center[0] + this->Axis[0]*off,
    this->Center[1] + this->Axis[1]*off,
    this->Center[2] + this->Axis[2]*off
  };

  // start/end points
  double pStart[3] = {
    this->Center[0] + this->StartDir[0]*this->Radius + this->Axis[0]*off,
    this->Center[1] + this->StartDir[1]*this->Radius + this->Axis[1]*off,
    this->Center[2] + this->StartDir[2]*this->Radius + this->Axis[2]*off
  };

  double pEnd[3] = {
    this->Center[0] + endDir[0]*this->Radius + this->Axis[0]*off,
    this->Center[1] + endDir[1]*this->Radius + this->Axis[1]*off,
    this->Center[2] + endDir[2]*this->Radius + this->Axis[2]*off
  };
  
   if (this->AttachedRenderer && this->AttachedRenderer->GetActiveCamera())
  {
    double dop[3];
    this->AttachedRenderer->GetActiveCamera()->GetDirectionOfProjection(dop);
    if (vtkMath::Normalize(dop) > 0.0)
    {
      const double eps = std::max(this->Radius * 1e-4, 1e-6);

      c[0]     -= dop[0] * eps; c[1]     -= dop[1] * eps; c[2]     -= dop[2] * eps;
      pStart[0]-= dop[0] * eps; pStart[1]-= dop[1] * eps; pStart[2]-= dop[2] * eps;
      pEnd[0]  -= dop[0] * eps; pEnd[1]  -= dop[1] * eps; pEnd[2]  -= dop[2] * eps;
    }
  }
  
  this->StartLineSource->SetPoint1(c);
  this->StartLineSource->SetPoint2(pStart);
  this->StartLineSource->Modified();

  this->EndLineSource->SetPoint1(c);
  this->EndLineSource->SetPoint2(pEnd);
  this->EndLineSource->Modified();

  // vtkArcSource wants positive angle; flip normal when negative
  double n[3] = { this->Axis[0], this->Axis[1], this->Axis[2] };
  if (angleDeg < 0.0)
  {
    angleDeg = -angleDeg;
    n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2];
  }

  // polar vector = start direction scaled by radius
  double polar[3] = {
    this->StartDir[0]*this->Radius,
    this->StartDir[1]*this->Radius,
    this->StartDir[2]*this->Radius
  };

   this->ArcSource->SetCenter(c);
  this->ArcSource->SetNormal(n);
  this->ArcSource->SetPolarVector(polar);
  this->ArcSource->SetAngle(angleDeg);
  this->ArcSource->SetResolution(this->ArcSegments);
  this->ArcSource->Modified();
  
    // Force ArcSource to produce updated points
  this->ArcSource->Update();

  vtkPolyData* arcPd = this->ArcSource->GetOutput();
  vtkPoints* arcPts = arcPd ? arcPd->GetPoints() : nullptr;
  const vtkIdType nArcPts = arcPts ? arcPts->GetNumberOfPoints() : 0;

  if (nArcPts >= 2 && this->FillPoints && this->FillPolys && this->FillPolyData)
  {
    // polygon = [center, arcPoint0, arcPoint1, ... arcPointN]
    this->FillPoints->Reset();
    this->FillPolys->Reset();

    this->FillPoints->SetNumberOfPoints(nArcPts + 1);
    this->FillPoints->SetPoint(0, c); // center with same depth offset

    for (vtkIdType i = 0; i < nArcPts; ++i)
    {
      double p[3];
      arcPts->GetPoint(i, p);
      this->FillPoints->SetPoint(i + 1, p);
    }

    vtkSmartPointer<vtkPolygon> poly = vtkSmartPointer<vtkPolygon>::New();
    poly->GetPointIds()->SetNumberOfIds(nArcPts + 1);

    for (vtkIdType i = 0; i < nArcPts + 1; ++i)
    {
      poly->GetPointIds()->SetId(i, i);
    }

    this->FillPolys->InsertNextCell(poly);

    this->FillPoints->Modified();
    this->FillPolys->Modified();
    this->FillPolyData->Modified();

    // optional but makes updates immediate/predictable
    this->FillTri->Update();
  }

}


double vtkRotationArcOverlay::Clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

void vtkRotationArcOverlay::Normalize3(double v[3])
{
  const double n2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (n2 <= 0.0)
  {
    v[0] = 1.0; v[1] = 0.0; v[2] = 0.0;
    return;
  }
  const double inv = 1.0 / std::sqrt(n2);
  v[0] *= inv; v[1] *= inv; v[2] *= inv;
}

void vtkRotationArcOverlay::Cross3(const double a[3], const double b[3], double out[3])
{
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

double vtkRotationArcOverlay::Dot3(const double a[3], const double b[3])
{
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vtkRotationArcOverlay::RotateRodrigues(const double v[3], const double axisUnit[3], double angleRad, double out[3])
{
  // Rodrigues: v' = v cos a + (k x v) sin a + k (k·v) (1 - cos a)
  const double c = std::cos(angleRad);
  const double s = std::sin(angleRad);

  double kxv[3];
  Cross3(axisUnit, v, kxv);

  const double kdv = Dot3(axisUnit, v);

  out[0] = v[0]*c + kxv[0]*s + axisUnit[0]*kdv*(1.0 - c);
  out[1] = v[1]*c + kxv[1]*s + axisUnit[1]*kdv*(1.0 - c);
  out[2] = v[2]*c + kxv[2]*s + axisUnit[2]*kdv*(1.0 - c);
  Normalize3(out);
}


vtkStandardNewMacro(vtkDepthOnlyActor);

        vtkStandardNewMacro(vtkTransformationWidget);

        //------------------------------------------------------------------------------
        vtkTransformationWidget::vtkTransformationWidget()
        {
        this->Placed = 0;

        this->InitialBounds[0] = this->InitialBounds[2] = this->InitialBounds[4] =  1.0;
        this->InitialBounds[1] = this->InitialBounds[3] = this->InitialBounds[5] = -1.0;

        this->EventCallbackCommand = vtkSmartPointer<vtkCallbackCommand>::New();
        this->EventCallbackCommand->SetCallback(vtkTransformationWidget::ProcessEvents);
        this->EventCallbackCommand->SetClientData(this);

        this->AxesAssembly  = vtkSmartPointer<vtkAssembly>::New();
        this->AxesTransform = vtkSmartPointer<vtkTransform>::New();
        this->ScaleTransform = vtkSmartPointer<vtkTransform>::New();

        this->ArrowXActor = nullptr;
        this->ArrowYActor = nullptr;
        this->ArrowZActor = nullptr;
        this->RotXActor   = nullptr;
        this->RotYActor   = nullptr;
        this->RotZActor   = nullptr;
        this->QuadXYActor = nullptr;
        this->QuadXZActor = nullptr;
        this->QuadZYActor = nullptr;
        this->ArrowXLineActor = nullptr;
        this->ArrowYLineActor = nullptr;
        this->ArrowZLineActor = nullptr;
        this->ArcXActor = nullptr;
        this->ArcYActor = nullptr;
        this->ArcZActor = nullptr;
        this->CenterSphereActor = nullptr;
        this->YModeLineActor = nullptr;

        // Interaction state
        arrowXPicked = arrowYPicked = arrowZPicked = false;
        quadXYPicked = quadXZPicked = quadZYPicked = false;
        rotXPicked = rotYPicked = rotZPicked = false;
        isAxisDragging = isPlaneDragging = isRotDragging = isCenterDragging = false;
        centerPicked = false;

        this->LeftButtonPressCallbackId  = 0;
        this->LeftButtonReleaseCallbackId = 0;
        this->MouseMoveCallbackId        = 0;
        this->CameraModifiedCallbackId   = 0;

        startMouseX = startMouseY = 0;
        m_rotationTheta = 0.0;
        LastPickPosition[0] = LastPickPosition[1] = LastPickPosition[2] = 0.0;
        m_pickedPointOffset[0] = m_pickedPointOffset[1] = m_pickedPointOffset[2] = 0.0;

        widgetCS.SetOrigin(0.0, 0.0, 0.0);
        widgetCS.SetDirX(1.0, 0.0, 0.0);
        widgetCS.SetDirY(0.0, 1.0, 0.0);
        widgetCS.SetDirZ(0.0, 0.0, 1.0);
        }
        //------------------------------------------------------------------------------
        vtkTransformationWidget::~vtkTransformationWidget()
        {
            SetEnabled(false);
            AxesTransform = nullptr;
            ScaleTransform = nullptr;
            AxesAssembly = nullptr;
            ArrowXActor = nullptr;
            ArrowYActor = nullptr;
            ArrowZActor = nullptr;
            RotXActor = nullptr;
            RotYActor = nullptr;
            RotZActor = nullptr;
            QuadXYActor = nullptr;
            QuadXZActor = nullptr;
            QuadZYActor = nullptr;
            ArrowXLineActor = nullptr;
            ArrowYLineActor = nullptr;
            ArrowZLineActor = nullptr;
            ArcXActor = nullptr;
            ArcYActor = nullptr;
            ArcZActor = nullptr;
            CenterSphereActor = nullptr;
            YModeLineActor = nullptr;
            UnregisterAllEvents();
        }

        //------------------------------------------------------------------------------
        void vtkTransformationWidget::PrintSelf(ostream& os, vtkIndent indent)
        {
            this->Superclass::PrintSelf(os, indent);
            os << indent << "vtkTransformationWidget with 3 separate arrow actors\n";
        }
		//------------------------------------------------------------------------------
		void vtkTransformationWidget::AttachTranslationOverlayToRenderer(vtkRenderer* ren)
		{
			if (!ren)
			{
				return;
			}

			if (this->TranslationLineActor && !ren->HasViewProp(this->TranslationLineActor))
			{
				ren->AddViewProp(this->TranslationLineActor);
			}
			if (this->TranslationTextActor && !ren->HasViewProp(this->TranslationTextActor))
			{
				ren->AddViewProp(this->TranslationTextActor);
			}
			if (this->TranslationStartMarkerActor && !ren->HasViewProp(this->TranslationStartMarkerActor))
			{
				ren->AddViewProp(this->TranslationStartMarkerActor);
			}
		}
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::DetachTranslationOverlayFromRenderer(vtkRenderer* ren)
		{
			if (!ren)
			{
				return;
			}

			if (this->TranslationLineActor && ren->HasViewProp(this->TranslationLineActor))
			{
				ren->RemoveViewProp(this->TranslationLineActor);
			}
			if (this->TranslationTextActor && ren->HasViewProp(this->TranslationTextActor))
			{
				ren->RemoveViewProp(this->TranslationTextActor);
			}
			if (this->TranslationStartMarkerActor && ren->HasViewProp(this->TranslationStartMarkerActor))
			{
				ren->RemoveViewProp(this->TranslationStartMarkerActor);
			}
		}
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetEnabled(int enabling)
		{
		  if (enabling)
		  {
			if (this->Enabled)
			{
			  return;
			}

			vtkRenderer* ren = this->m_frontRenderer;
			if (!ren)
			{
			  return;
			}

			this->AttachRotationOverlayToRenderer(ren);
			this->SetDefaultRenderer(ren);
			this->SetCurrentRenderer(ren);

			// Build once (no rebuilds)
			if (!this->ArrowXActor || !this->ArrowYActor || !this->ArrowZActor)
			{
			  this->CreateDefaultRepresentation();
			}

			// Ensure overlay props exist
			this->CreateTranslationOverlayIfNeeded();
			this->CreateRotationOverlayIfNeeded();

			this->RotationArcOverlay.AttachToRenderer(ren);
			this->RotationArcOverlay.End();


			// Start hidden
			this->EndTranslationOverlay();
			this->EndRotationOverlay();
			this->EndRotationArcOverlay();

			// Attach overlays FIRST
			if (this->TranslationLineActor && !ren->HasViewProp(this->TranslationLineActor))
			{
			  ren->AddViewProp(this->TranslationLineActor);
			}
			if (this->TranslationTextActor && !ren->HasViewProp(this->TranslationTextActor))
			{
			  ren->AddViewProp(this->TranslationTextActor);
			}
			if (this->TranslationStartMarkerActor && !ren->HasViewProp(this->TranslationStartMarkerActor))
			{
			  ren->AddViewProp(this->TranslationStartMarkerActor);
			}
			if (this->RotationTextActor && !ren->HasViewProp(this->RotationTextActor))
			{
			  ren->AddViewProp(this->RotationTextActor);
			}

			// Attach widget assembly ONCE
			if (this->AxesAssembly && !ren->HasViewProp(this->AxesAssembly))
			{
			  ren->AddViewProp(this->AxesAssembly);
			}

			this->ApplyCallbacks();
			this->Init();
			this->UpdateWidgetGeometry();

			this->Enabled = 1;

			if (ren->GetRenderWindow())
			{
			  ren->GetRenderWindow()->Render();
			}

			return;
		  }

		  // disabling
		  if (!this->Enabled)
		  {
			return;
		  }

		  this->UnregisterAllEvents();

		  this->EndRotationArcOverlay();
		  this->EndRotationOverlay();
		  this->EndTranslationOverlay();

		  // Remove from BOTH renderers defensively 
		  vtkRenderer* renFront = this->m_frontRenderer;

		  if (renFront)
		  {
			if (this->AxesAssembly && renFront->HasViewProp(this->AxesAssembly))
			{
			  renFront->RemoveViewProp(this->AxesAssembly);
			}
			if (this->TranslationLineActor && renFront->HasViewProp(this->TranslationLineActor))
			{
			  renFront->RemoveViewProp(this->TranslationLineActor);
			}
			if (this->TranslationTextActor && renFront->HasViewProp(this->TranslationTextActor))
			{
			  renFront->RemoveViewProp(this->TranslationTextActor);
			}
			if (this->TranslationStartMarkerActor && renFront->HasViewProp(this->TranslationStartMarkerActor))
			{
			  renFront->RemoveViewProp(this->TranslationStartMarkerActor);
			}
			if (this->RotationTextActor && renFront->HasViewProp(this->RotationTextActor))
			{
			  renFront->RemoveViewProp(this->RotationTextActor);
			}
		  }


		  this->AttachRotationOverlayToRenderer(renFront);
		  this->RotationArcOverlay.End();
		  this->RotationArcOverlay.DetachFromRenderer();

		  this->ReleaseFocus();
		  this->Enabled = 0;

		  vtkRenderer* ren = renFront;
		  if (ren && ren->GetRenderWindow())
		  {
			ren->GetRenderWindow()->Render();
		  }
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::ApplyCallbacks()
        {    
            if (this->GetInteractor())
            {
				this->LeftButtonPressCallbackId =
					this->GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent,
													   this->EventCallbackCommand, 1.0f);

				this->LeftButtonReleaseCallbackId =
					this->GetInteractor()->AddObserver(vtkCommand::LeftButtonReleaseEvent,
													   this->EventCallbackCommand, 1.0f);

				this->MouseMoveCallbackId =
					this->GetInteractor()->AddObserver(vtkCommand::MouseMoveEvent,
													   this->EventCallbackCommand, 1.0f);
            }

            if (this->GetCurrentRenderer() && this->GetCurrentRenderer()->GetActiveCamera())
            {
                vtkCamera* cam = this->GetCurrentRenderer()->GetActiveCamera();
                this->CameraModifiedCallbackId =
                    cam->AddObserver(vtkCommand::ModifiedEvent,
                        this,
                        &vtkTransformationWidget::UpdateScaleToCamera);
						
            }
        }
        //------------------------------------------------------------------------------		
		void vtkTransformationWidget::BeginRotationArcOverlay()
		{
		  vtkRenderer* ren = this->GetCurrentRenderer();
		  if (!ren) { return; }

		  double axisW[3];
		  if (rotXPicked) { this->widgetCSStart.GetX(axisW); }
		  else if (rotYPicked) { this->widgetCSStart.GetY(axisW); }
		  else { this->widgetCSStart.GetZ(axisW); }

		  if (vtkMath::Normalize(axisW) == 0.0) { return; }

		  double centerW[3];
		  this->widgetCSStart.GetOrigin(centerW);

		  // start vector (world): center -> pick
		  double v[3] = {
			this->LastPickPosition[0] - centerW[0],
			this->LastPickPosition[1] - centerW[1],
			this->LastPickPosition[2] - centerW[2]
		  };

		  // project into disc plane
		  const double d = vtkMath::Dot(v, axisW);
		  v[0] -= d*axisW[0]; v[1] -= d*axisW[1]; v[2] -= d*axisW[2];

		  const double radius = vtkMath::Normalize(v); // normalizes v and returns length
		  if (radius <= 1e-9) { return; }

		  this->RotationArcOverlay.AttachToRenderer(ren);
		  this->RotationArcOverlay.SetColor(1.0, 1.0, 1.0);
		  this->RotationArcOverlay.SetLineWidth(3.0);
		  this->RotationArcOverlay.SetArcSegments(64);
		  this->RotationArcOverlay.SetDepthOffsetLocal(0.0);

		  this->RotationArcOverlay.Begin(axisW, v, centerW, radius);
		}
        //------------------------------------------------------------------------------	
		void vtkTransformationWidget::UpdateRotationArcOverlayFromHitWorld(const double hitWorld[3])
		{
		  if (!this->RotationArcOverlay.IsActive()) { return; }

		  double axisW[3];
		  if (rotXPicked) { this->widgetCSStart.GetX(axisW); }
		  else if (rotYPicked) { this->widgetCSStart.GetY(axisW); }
		  else { this->widgetCSStart.GetZ(axisW); }

		  if (vtkMath::Normalize(axisW) == 0.0) { return; }

		  double centerW[3];
		  this->widgetCSStart.GetOrigin(centerW);

		  double v[3] = {
			hitWorld[0] - centerW[0],
			hitWorld[1] - centerW[1],
			hitWorld[2] - centerW[2]
		  };

		  const double d = vtkMath::Dot(v, axisW);
		  v[0] -= d*axisW[0]; v[1] -= d*axisW[1]; v[2] -= d*axisW[2];

		  if (vtkMath::Normalize(v) == 0.0) { return; }

		  this->RotationArcOverlay.Update(v);
		}
        //------------------------------------------------------------------------------	
		void vtkTransformationWidget::EndRotationArcOverlay()
		{
		  this->RotationArcOverlay.End();
		}
        //------------------------------------------------------------------------------		
		bool vtkTransformationWidget::WorldToWidgetLocal(const double pWorld[3], double pLocal[3]) const{
		  if (!this->AxesAssembly)
		  {
			return false;
		  }

		  vtkLinearTransform* lt = this->AxesAssembly->GetUserTransform();
		  if (!lt || !lt->GetMatrix())
		  {
			return false;
		  }

		  vtkNew<vtkMatrix4x4> inv;
		  vtkMatrix4x4::Invert(lt->GetMatrix(), inv);

		  double pw[4] = { pWorld[0], pWorld[1], pWorld[2], 1.0 };
		  double pl[4] = { 0.0, 0.0, 0.0, 1.0 };
		  inv->MultiplyPoint(pw, pl);

		  pLocal[0] = pl[0];
		  pLocal[1] = pl[1];
		  pLocal[2] = pl[2];
		  return true;
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::UnregisterAllEvents()
        {
            // Interactor-based observers
            if (this->GetInteractor())
            {
                if (this->LeftButtonPressCallbackId != 0)
                {
                    this->GetInteractor()->RemoveObserver(this->LeftButtonPressCallbackId);
                    this->LeftButtonPressCallbackId = 0;
                }

                if (this->LeftButtonReleaseCallbackId != 0)
                {
                    this->GetInteractor()->RemoveObserver(this->LeftButtonReleaseCallbackId);
                    this->LeftButtonReleaseCallbackId = 0;
                }

                if (this->MouseMoveCallbackId != 0)
                {
                    this->GetInteractor()->RemoveObserver(this->MouseMoveCallbackId);
                    this->MouseMoveCallbackId = 0;
                }
            }

            // Camera-based observers
            if (this->GetCurrentRenderer() && this->GetCurrentRenderer()->GetActiveCamera())
            {
                vtkCamera* cam = this->GetCurrentRenderer()->GetActiveCamera();
                if (this->CameraModifiedCallbackId != 0)
                {
                    cam->RemoveObserver(this->CameraModifiedCallbackId);
                    this->CameraModifiedCallbackId = 0;
                }
            }
        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetArcsAndLinesVisibility(bool arcX, bool arcY, bool arcZ,
															   bool lineX, bool lineY, bool lineZ)
		{
			if (this->ArcXActor) {
				this->ArcXActor->SetVisibility((arcX && RotXActive) ? 1 : 0);
			}
			if (this->ArcYActor) {
				this->ArcYActor->SetVisibility(((arcY && RotYActive) || YDiscVisible) ? 1 : 0);
			}
			if (this->ArcZActor) {
				this->ArcZActor->SetVisibility((arcZ && RotZActive) ? 1 : 0);
			}

			if (this->ArrowXLineActor) {
				this->ArrowXLineActor->SetVisibility((lineX && this->AxisLineXVisible) ? 1 : 0);
			}
			if (this->ArrowYLineActor) {
				this->ArrowYLineActor->SetVisibility((lineY && this->AxisLineYVisible) ? 1 : 0);
			}
			if (this->ArrowZLineActor) {
				this->ArrowZLineActor->SetVisibility((lineZ && this->AxisLineZVisible) ? 1 : 0);
			}

			if (this->YModeLineActor) {
				this->YModeLineActor->SetVisibility((this->AxisLineYVisible && this->YDiscVisible) ? 1 : 0);
			}
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetHandlesVisibility(bool arrowX, bool arrowY, bool arrowZ,
            bool quadXY, bool quadXZ, bool quadZY, bool rotX, bool rotY, bool rotZ, bool center)
        {
            if (this->ArrowXActor)
                this->ArrowXActor->SetVisibility(arrowX && ArrowXActive);
            if (this->ArrowYActor)
                this->ArrowYActor->SetVisibility(arrowY && ArrowYActive);
            if (this->ArrowZActor)
                this->ArrowZActor->SetVisibility(arrowZ && ArrowZActive);

            if (this->QuadXYActor)
                this->QuadXYActor->SetVisibility(quadXY && QuadXYActive);
            if (this->QuadXZActor)
                this->QuadXZActor->SetVisibility(quadXZ && QuadXZActive);
            if (this->QuadZYActor)
                this->QuadZYActor->SetVisibility(quadZY && QuadZYActive);

            if (this->RotXActor)
                this->RotXActor->SetVisibility(rotX && RotXActive);
            if (this->RotYActor)
                this->RotYActor->SetVisibility(rotY && RotYActive);
            if (this->RotZActor)
                this->RotZActor->SetVisibility(rotZ && RotZActive );
            if (this->CenterSphereActor)
                this->CenterSphereActor->SetVisibility(center && CenterActive);
			

        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::CreateDefaultRepresentation()
		{
		  if (!this->AxesAssembly) {
			this->AxesAssembly = vtkSmartPointer<vtkAssembly>::New();
		  }

		  // ---------------------------------------------------------------------------
		  // 1) CREATE ALL PROPS 
		  // ---------------------------------------------------------------------------
		  this->CreateCenterSphere(this->CompassCenterSphereRadius);

		  this->CreateXArrow(this->CompassArrowTipLength, this->CompassArrowTipRadius, this->CompassArrowShaftRadius,
							 this->CompassArrowTipRes, this->CompassArrowShaftRes, this->CompassArrowOffset);
		  this->CreateYArrow(this->CompassArrowTipLength, this->CompassArrowTipRadius, this->CompassArrowShaftRadius,
							 this->CompassArrowTipRes, this->CompassArrowShaftRes, this->CompassArrowOffset);
		  this->CreateZArrow(this->CompassArrowTipLength, this->CompassArrowTipRadius, this->CompassArrowShaftRadius,
							 this->CompassArrowTipRes, this->CompassArrowShaftRes, this->CompassArrowOffset);

		  this->CreateXLine(this->CompassMaxLineExtent);
		  this->CreateYLine(this->CompassMaxLineExtent);
		  this->CreateZLine(this->CompassMaxLineExtent);

		  this->CreateXYQuad();
		  this->CreateXZQuad();
		  this->CreateZYQuad();

		  this->CreateXDisc(this->CompassRotHandleRadius);
		  this->CreateYDisc(this->CompassRotHandleRadius);
		  this->CreateZDisc(this->CompassRotHandleRadius);

		  this->CreateRotXHandle(this->CompassRotHandleRadius);
		  this->CreateRotYHandle(this->CompassRotHandleRadius);
		  this->CreateRotZHandle(this->CompassRotHandleRadius);

		  this->CreateOcclusionSphere(this->CompassRotHandleRadius);



		  this->CreateTranslationOverlayIfNeeded(); 
		  this->CreateRotationOverlayIfNeeded();   

		  // ---------------------------------------------------------------------------
		  // 2) ADD PARTS TO ASSEMBLY 
		  //    Order matters for depth-only occlusion trick.
		  // ---------------------------------------------------------------------------

		  // Depth pre-pass MUST be first so it writes depth before other widget geometry.
			if (this->YModeLineActor) { this->AxesAssembly->AddPart(this->YModeLineActor); }

			if (this->ArrowXLineActor) { this->AxesAssembly->AddPart(this->ArrowXLineActor); }
			if (this->ArrowYLineActor) { this->AxesAssembly->AddPart(this->ArrowYLineActor); }
			if (this->ArrowZLineActor) { this->AxesAssembly->AddPart(this->ArrowZLineActor); }

			if (this->ArrowXActor) { this->AxesAssembly->AddPart(this->ArrowXActor); }
			if (this->ArrowYActor) { this->AxesAssembly->AddPart(this->ArrowYActor); }
			if (this->ArrowZActor) { this->AxesAssembly->AddPart(this->ArrowZActor); }

			if (this->QuadXYActor) { this->AxesAssembly->AddPart(this->QuadXYActor); }
			if (this->QuadXZActor) { this->AxesAssembly->AddPart(this->QuadXZActor); }
			if (this->QuadZYActor) { this->AxesAssembly->AddPart(this->QuadZYActor); }
			
			if (this->ArcXActor) { this->AxesAssembly->AddPart(this->ArcXActor); }
			if (this->ArcYActor) { this->AxesAssembly->AddPart(this->ArcYActor); }
			if (this->ArcZActor) { this->AxesAssembly->AddPart(this->ArcZActor); }	

			if (this->CenterSphereActor) { this->AxesAssembly->AddPart(this->CenterSphereActor); }

			if (this->OcclusionSphereDepthActor) { this->AxesAssembly->AddPart(this->OcclusionSphereDepthActor); }

			// Things you want occluded by the sphere depth
			if (this->RotXActor) { this->AxesAssembly->AddPart(this->RotXActor); }
			if (this->RotYActor) { this->AxesAssembly->AddPart(this->RotYActor); }
			if (this->RotZActor) { this->AxesAssembly->AddPart(this->RotZActor); }

			// Visual translucent sphere last
			if (this->OcclusionSphereActor) { this->AxesAssembly->AddPart(this->OcclusionSphereActor); }

		  // ---------------------------------------------------------------------------
		  // 3)  enforce solid colors 
		  // ---------------------------------------------------------------------------
		  auto ForceSolidColor = [](vtkActor* a)
		  {
			if (!a) { return; }
			vtkMapper* m = a->GetMapper();
			if (!m) { return; }
			m->ScalarVisibilityOff();
		  };

		  ForceSolidColor(this->ArrowXActor);
		  ForceSolidColor(this->ArrowYActor);
		  ForceSolidColor(this->ArrowZActor);
		  ForceSolidColor(this->ArrowXLineActor);
		  ForceSolidColor(this->ArrowYLineActor);
		  ForceSolidColor(this->ArrowZLineActor);
		  ForceSolidColor(this->YModeLineActor);
		  ForceSolidColor(this->QuadXYActor);
		  ForceSolidColor(this->QuadXZActor);
		  ForceSolidColor(this->QuadZYActor);
		  ForceSolidColor(this->RotXActor);
		  ForceSolidColor(this->RotYActor);
		  ForceSolidColor(this->RotZActor);
		  ForceSolidColor(this->ArcXActor);
		  ForceSolidColor(this->ArcYActor);
		  ForceSolidColor(this->ArcZActor);
		  ForceSolidColor(this->CenterSphereActor);
		  ForceSolidColor(this->OcclusionSphereActor);

		  this->AxesAssembly->SetScale(0.7);
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateZLine(double MaxLineExtent)
        {
            vtkSmartPointer<vtkLineSource> lineSourceZ = vtkSmartPointer<vtkLineSource>::New();
            lineSourceZ->SetPoint1(0.0, 0.0, -MaxLineExtent);
            lineSourceZ->SetPoint2(0.0, 0.0, +MaxLineExtent);

            vtkSmartPointer<vtkPolyDataMapper> lineMapperZ = vtkSmartPointer<vtkPolyDataMapper>::New();
            lineMapperZ->SetInputConnection(lineSourceZ->GetOutputPort());

            this->ArrowZLineActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowZLineActor->SetMapper(lineMapperZ);
            this->ArrowZLineActor->GetProperty()->SetColor(Z_HOVER_COLOR[0], Z_HOVER_COLOR[1], Z_HOVER_COLOR[2]);
            this->ArrowZLineActor->GetProperty()->SetLineWidth(HelperLineWidth);
            this->ArrowZLineActor->SetVisibility(ArrowZActive);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateCenterSphere(double radius)
        {
            vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
            sphereSource->SetCenter(0.0, 0.0, 0.0); 
            sphereSource->SetRadius(radius);         
			sphereSource->SetPhiResolution(this->CompassSpherePhiResolution);
			sphereSource->SetThetaResolution(this->CompassSphereThetaResolution);

            vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

            this->CenterSphereActor = vtkSmartPointer<vtkActor>::New();
            this->CenterSphereActor->SetMapper(sphereMapper);
            this->CenterSphereActor->GetProperty()->SetColor(1.0, 1.0, 1.0);  
            this->CenterSphereActor->SetVisibility(true);

            this->CenterSphereActor->GetProperty()->SetAmbientColor(1.0, 1.0, 1.0);
            this->CenterSphereActor->GetProperty()->SetAmbient(1.0);
            this->CenterSphereActor->GetProperty()->SetDiffuse(0.0);
            this->CenterSphereActor->GetProperty()->SetSpecular(0.0);
            this->CenterSphereActor->GetProperty()->SetInterpolationToFlat();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateYLine(double MaxLineExtent)
        {
            vtkSmartPointer<vtkLineSource> lineSourceY = vtkSmartPointer<vtkLineSource>::New();
            lineSourceY->SetPoint1(0.0, -MaxLineExtent, 0.0);
            lineSourceY->SetPoint2(0.0, +MaxLineExtent, 0.0);

            vtkSmartPointer<vtkPolyDataMapper> lineMapperY = vtkSmartPointer<vtkPolyDataMapper>::New();
            lineMapperY->SetInputConnection(lineSourceY->GetOutputPort());

            this->ArrowYLineActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowYLineActor->SetMapper(lineMapperY);
            this->ArrowYLineActor->GetProperty()->SetColor(Y_HOVER_COLOR[0], Y_HOVER_COLOR[1], Y_HOVER_COLOR[2]);
            this->ArrowYLineActor->GetProperty()->SetLineWidth(HelperLineWidth);
            this->ArrowYLineActor->SetVisibility(ArrowYActive);


            lineSourceY = vtkSmartPointer<vtkLineSource>::New();
            lineSourceY->SetPoint1(0.0, -0.4, 0.0);
            lineSourceY->SetPoint2(0.0, +0.4, 0.0);

            lineMapperY = vtkSmartPointer<vtkPolyDataMapper>::New();
            lineMapperY->SetInputConnection(lineSourceY->GetOutputPort());

            this->YModeLineActor = vtkSmartPointer<vtkActor>::New();
            this->YModeLineActor->SetMapper(lineMapperY);
            this->YModeLineActor->GetProperty()->SetColor(Y_HOVER_COLOR[0], Y_HOVER_COLOR[1], Y_HOVER_COLOR[2]);
            this->YModeLineActor->GetProperty()->SetLineWidth(2.0);
            this->YModeLineActor->SetVisibility(YDiscVisible);

        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateXLine(double MaxLineExtent)
        {
            vtkSmartPointer<vtkLineSource> lineSourceX = vtkSmartPointer<vtkLineSource>::New();
            lineSourceX->SetPoint1(-MaxLineExtent, 0.0, 0.0);
            lineSourceX->SetPoint2(+MaxLineExtent, 0.0, 0.0);

            // 3D mapper
            vtkSmartPointer<vtkPolyDataMapper> lineMapperX = vtkSmartPointer<vtkPolyDataMapper>::New();
            lineMapperX->SetInputConnection(lineSourceX->GetOutputPort());

            // 3D actor
            this->ArrowXLineActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowXLineActor->SetMapper(lineMapperX);
            this->ArrowXLineActor->GetProperty()->SetLineWidth(HelperLineWidth);
            this->ArrowXLineActor->SetVisibility(ArrowXActive);
            this->ArrowXLineActor->GetProperty()->SetColor(X_HOVER_COLOR[0], X_HOVER_COLOR[1], X_HOVER_COLOR[2]);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateZArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset)
        {
            vtkSmartPointer<vtkArrowSource> arrowSourceZ = vtkSmartPointer<vtkArrowSource>::New();
            arrowSourceZ->SetTipLength(tipLength);
            arrowSourceZ->SetTipRadius(tipRadius);
            arrowSourceZ->SetShaftRadius(shaftRadius);
            arrowSourceZ->SetTipResolution(tipRes);
            arrowSourceZ->SetShaftResolution(shaftRes);

            vtkSmartPointer<vtkTriangleFilter> triangleFilterZ = vtkSmartPointer<vtkTriangleFilter>::New();
            triangleFilterZ->SetInputConnection(arrowSourceZ->GetOutputPort());
            triangleFilterZ->Update();

            vtkSmartPointer<vtkPolyDataMapper> arrowMapperZ = vtkSmartPointer<vtkPolyDataMapper>::New();
            arrowMapperZ->SetInputConnection(triangleFilterZ->GetOutputPort());

            this->ArrowZActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowZActor->SetMapper(arrowMapperZ);
            this->ArrowZActor->GetProperty()->SetColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            this->ArrowZActor->SetScale(this->CompassArrowScale, 1.0, 1.0);
            this->ArrowZActor->RotateY(-90.0);
            this->ArrowZActor->SetPosition(0.0, 0.0, arrowOffset);
            this->ArrowZActor->GetProperty()->SetAmbientColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            this->ArrowZActor->GetProperty()->SetAmbient(1.0);
            this->ArrowZActor->GetProperty()->SetDiffuse(0.0);
            this->ArrowZActor->SetVisibility(ArrowZActive);
            this->ArrowZActor->GetProperty()->SetSpecular(0.6);
            this->ArrowZActor->GetProperty()->SetSpecularPower(50.0);
            this->ArrowZActor->GetProperty()->SetInterpolationToPhong();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateYArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset)
        {
            vtkSmartPointer<vtkArrowSource> arrowSourceY = vtkSmartPointer<vtkArrowSource>::New();
            arrowSourceY->SetTipLength(tipLength);
            arrowSourceY->SetTipRadius(tipRadius);
            arrowSourceY->SetShaftRadius(shaftRadius);
            arrowSourceY->SetTipResolution(tipRes);
            arrowSourceY->SetShaftResolution(shaftRes);

            vtkSmartPointer<vtkTriangleFilter> triangleFilterY = vtkSmartPointer<vtkTriangleFilter>::New();
            triangleFilterY->SetInputConnection(arrowSourceY->GetOutputPort());
            triangleFilterY->Update();

            vtkSmartPointer<vtkPolyDataMapper> arrowMapperY = vtkSmartPointer<vtkPolyDataMapper>::New();
            arrowMapperY->SetInputConnection(triangleFilterY->GetOutputPort());

            this->ArrowYActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowYActor->SetMapper(arrowMapperY);
            this->ArrowYActor->GetProperty()->SetColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]); 
            this->ArrowYActor->SetScale(this->CompassArrowScale, 1.0, 1.0);
            this->ArrowYActor->RotateZ(90.0);
            this->ArrowYActor->SetPosition(0.0, arrowOffset, 0.0);
            this->ArrowYActor->GetProperty()->SetAmbientColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]);
            this->ArrowYActor->GetProperty()->SetAmbient(1.0);
            this->ArrowYActor->SetVisibility(ArrowYActive);
            this->ArrowYActor->GetProperty()->SetDiffuse(0.0);
            this->ArrowYActor->GetProperty()->SetSpecular(0.0);
            this->ArrowYActor->GetProperty()->SetInterpolationToFlat();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateXArrow(double tipLength, double tipRadius, double shaftRadius, int tipRes, int shaftRes, double arrowOffset)
        {
            vtkSmartPointer<vtkArrowSource> arrowSourceX = vtkSmartPointer<vtkArrowSource>::New();
            arrowSourceX->SetTipLength(tipLength);
            arrowSourceX->SetTipRadius(tipRadius);
            arrowSourceX->SetShaftRadius(shaftRadius);
            arrowSourceX->SetTipResolution(tipRes);
            arrowSourceX->SetShaftResolution(shaftRes);

            vtkSmartPointer<vtkTriangleFilter> triangleFilterX = vtkSmartPointer<vtkTriangleFilter>::New();
            triangleFilterX->SetInputConnection(arrowSourceX->GetOutputPort());
            triangleFilterX->Update();

            vtkSmartPointer<vtkPolyDataMapper> arrowMapperX = vtkSmartPointer<vtkPolyDataMapper>::New();
            arrowMapperX->SetInputConnection(triangleFilterX->GetOutputPort());

            this->ArrowXActor = vtkSmartPointer<vtkActor>::New();
            this->ArrowXActor->SetMapper(arrowMapperX);
            this->ArrowXActor->GetProperty()->SetColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
            this->ArrowXActor->SetScale(this->CompassArrowScale, 1.0, 1.0);
            this->ArrowXActor->SetPosition(arrowOffset, 0.0, 0.0);
            this->ArrowXActor->GetProperty()->SetAmbientColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
            this->ArrowXActor->GetProperty()->SetAmbient(1.0);
            this->ArrowXActor->SetVisibility(ArrowXActive);
            this->ArrowXActor->GetProperty()->SetDiffuse(0.0);
            this->ArrowXActor->GetProperty()->SetSpecular(0.0);
            this->ArrowXActor->GetProperty()->SetInterpolationToFlat();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateXYQuad()
        {
            vtkSmartPointer<vtkPlaneSource> planeSourceXY = vtkSmartPointer<vtkPlaneSource>::New();
            planeSourceXY->SetXResolution(1);
            planeSourceXY->SetYResolution(1);
			planeSourceXY->SetOrigin(this->CompassQuadMin, this->CompassQuadMin, 0.0);
			planeSourceXY->SetPoint1(this->CompassQuadMax, this->CompassQuadMin, 0.0);
			planeSourceXY->SetPoint2(this->CompassQuadMin, this->CompassQuadMax, 0.0);

            vtkSmartPointer<vtkPolyDataMapper> planeMapperXY = vtkSmartPointer<vtkPolyDataMapper>::New();
            planeMapperXY->SetInputConnection(planeSourceXY->GetOutputPort());

            this->QuadXYActor = vtkSmartPointer<vtkActor>::New();
            this->QuadXYActor->SetMapper(planeMapperXY);
            this->QuadXYActor->GetProperty()->SetColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            this->QuadXYActor->SetVisibility(QuadXYActive);
            this->QuadXYActor->GetProperty()->SetAmbientColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            this->QuadXYActor->GetProperty()->SetAmbient(1.0);
            this->QuadXYActor->GetProperty()->SetDiffuse(0.0);
            this->QuadXYActor->GetProperty()->SetSpecular(0.0);
        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::CreateXZQuad()
		{
			vtkSmartPointer<vtkPlaneSource> planeSourceXZ = vtkSmartPointer<vtkPlaneSource>::New();
			planeSourceXZ->SetXResolution(1);
			planeSourceXZ->SetYResolution(1);

			const double min = this->CompassQuadMin;
			const double max = this->CompassQuadMax;

			// XZ plane => Y is constant (0.0)
			planeSourceXZ->SetOrigin(min, 0.0, min);
			planeSourceXZ->SetPoint1(max, 0.0, min);
			planeSourceXZ->SetPoint2(min, 0.0, max);

			vtkSmartPointer<vtkPolyDataMapper> planeMapperXZ = vtkSmartPointer<vtkPolyDataMapper>::New();
			planeMapperXZ->SetInputConnection(planeSourceXZ->GetOutputPort());

			this->QuadXZActor = vtkSmartPointer<vtkActor>::New();
			this->QuadXZActor->SetMapper(planeMapperXZ);
			this->QuadXZActor->GetProperty()->SetColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]);
			this->QuadXZActor->SetVisibility(this->QuadXZActive);
			this->QuadXZActor->GetProperty()->SetAmbientColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]);
			this->QuadXZActor->GetProperty()->SetAmbient(1.0);
			this->QuadXZActor->GetProperty()->SetDiffuse(0.0);
			this->QuadXZActor->GetProperty()->SetSpecular(0.0);
		}
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::CreateZYQuad()
		{
			vtkSmartPointer<vtkPlaneSource> planeSourceZY = vtkSmartPointer<vtkPlaneSource>::New();
			planeSourceZY->SetXResolution(1);
			planeSourceZY->SetYResolution(1);

			const double min = this->CompassQuadMin;
			const double max = this->CompassQuadMax;

			// ZY plane => X is constant (0.0)
			planeSourceZY->SetOrigin(0.0, min, min);
			planeSourceZY->SetPoint1(0.0, max, min);
			planeSourceZY->SetPoint2(0.0, min, max);

			vtkSmartPointer<vtkPolyDataMapper> planeMapperZY = vtkSmartPointer<vtkPolyDataMapper>::New();
			planeMapperZY->SetInputConnection(planeSourceZY->GetOutputPort());

			this->QuadZYActor = vtkSmartPointer<vtkActor>::New();
			this->QuadZYActor->SetMapper(planeMapperZY);
			this->QuadZYActor->GetProperty()->SetColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
			this->QuadZYActor->SetVisibility(this->QuadZYActive);
			this->QuadZYActor->GetProperty()->SetAmbientColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
			this->QuadZYActor->GetProperty()->SetAmbient(1.0);
			this->QuadZYActor->GetProperty()->SetDiffuse(0.0);
			this->QuadZYActor->GetProperty()->SetSpecular(0.0);
		}
        //------------------------------------------------------------------------------
		
		static vtkLineSource* GetLineSourceFromActor(vtkActor* a)
{
    if (!a) { return nullptr; }

    vtkPolyDataMapper* m = vtkPolyDataMapper::SafeDownCast(a->GetMapper());
    if (!m) { return nullptr; }

    vtkAlgorithmOutput* inConn = m->GetInputConnection(0, 0);
    if (!inConn) { return nullptr; }

    return vtkLineSource::SafeDownCast(inConn->GetProducer());
}

static bool ClipInfiniteLineToFrustum( 
    vtkRenderer* ren, const double origin[3], 
    const double dirUnitIn[3],
    double out0[3], double out1[3])
{
  
  if (!ren) { return false; }
    vtkCamera* cam = ren->GetActiveCamera();
    if (!cam) { return false; }

    double dirUnit[3] = { dirUnitIn[0], dirUnitIn[1], dirUnitIn[2] };
    if (vtkMath::Normalize(dirUnit) == 0.0) { return false; }

    double cr[2];
    cam->GetClippingRange(cr); // cr[1] = far

    double camPos[3];
    cam->GetPosition(camPos);
    const double dist = std::sqrt(vtkMath::Distance2BetweenPoints(origin, camPos));

    const double L = std::max(cr[1], dist) * 2.0;

    out0[0] = origin[0] - dirUnit[0] * L;
    out0[1] = origin[1] - dirUnit[1] * L;
    out0[2] = origin[2] - dirUnit[2] * L;

    out1[0] = origin[0] + dirUnit[0] * L;
    out1[1] = origin[1] + dirUnit[1] * L;
    out1[2] = origin[2] + dirUnit[2] * L;

    return true;
}
		
void vtkTransformationWidget::UpdateLinePositions()
{
    vtkRenderer* ren = this->GetCurrentRenderer();
    if (!ren) { return; }

    vtkCamera* cam = ren->GetActiveCamera();
    if (!cam) { return; }
	
	double cr[2];
	//cam->GetClippingRange(cr);
	//cam->SetClippingRange(cr[0], 1e9);

    // Origin + axes in WORLD
    double oW[3];
    this->widgetCS.GetOrigin(oW);

    double xW[3], yW[3], zW[3];
    this->widgetCS.GetX(xW);
    this->widgetCS.GetY(yW);
    this->widgetCS.GetZ(zW);

    if (vtkMath::Normalize(xW) == 0.0) { return; }
    if (vtkMath::Normalize(yW) == 0.0) { return; }
    if (vtkMath::Normalize(zW) == 0.0) { return; }

    // Grab the actual line sources from the mapper connections
    vtkLineSource* lsX = GetLineSourceFromActor(this->ArrowXLineActor);
    vtkLineSource* lsY = GetLineSourceFromActor(this->ArrowYLineActor);
    vtkLineSource* lsZ = GetLineSourceFromActor(this->ArrowZLineActor);

    auto updateOne = [&](vtkLineSource* ls, const double axisW[3])
    {
        if (!ls) { return; }

        double p0W[3], p1W[3];
        if (!ClipInfiniteLineToFrustum(ren, oW, axisW, p0W, p1W))
        {
            return;
        }

        // Convert WORLD endpoints to widget LOCAL (because the actor lives inside AxesAssembly)
        double p0L[3], p1L[3];
        if (!this->WorldToWidgetLocal(p0W, p0L)) { return; }
        if (!this->WorldToWidgetLocal(p1W, p1L)) { return; }

        ls->SetPoint1(p0L);
        ls->SetPoint2(p1L);
        ls->Modified();
    };

    updateOne(lsX, xW);
    updateOne(lsY, yW);
    updateOne(lsZ, zW);

    if (ren->GetRenderWindow())
    {
        ren->GetRenderWindow()->Render();
    }
}

//------------------------------------------------------------------------------
void vtkTransformationWidget::PlaceWidget(double bounds[6])
{
  if (!bounds) return;

  double b[6] = {
    bounds[0], bounds[1],
    bounds[2], bounds[3],
    bounds[4], bounds[5]
  };

  if (b[0] > b[1]) std::swap(b[0], b[1]);
  if (b[2] > b[3]) std::swap(b[2], b[3]);
  if (b[4] > b[5]) std::swap(b[4], b[5]);

  const double cx = 0.5 * (b[0] + b[1]);
  const double cy = 0.5 * (b[2] + b[3]);
  const double cz = 0.5 * (b[4] + b[5]);

  for (int i = 0; i < 6; ++i) this->InitialBounds[i] = b[i];
  this->Placed = 1;

  if (this->AxesTransform)
  {
    this->AxesTransform->Identity();
    this->AxesTransform->Translate(cx, cy, cz);
    if (this->AxesAssembly)
      this->AxesAssembly->SetUserTransform(this->AxesTransform);
  }

  widgetCS.SetOrigin(cx, cy, cz);
  auto* ren = this->GetCurrentRenderer();
  if (!ren) return;

  UpdateWidgetGeometry();

  if (auto* rw = ren->GetRenderWindow())
    rw->Render();
}

void vtkTransformationWidget::PlaceWidgetToPoint(double x, double y, double z)
{
  if (this->AxesTransform)
  {
    this->AxesTransform->Identity();
    this->AxesTransform->Translate(x, y, z);

    if (this->AxesAssembly)
      this->AxesAssembly->SetUserTransform(this->AxesTransform);
  }

  widgetCS.SetOrigin(x, y, z);

  this->Placed = 1;
  this->InitialBounds[0] = this->InitialBounds[1] = x;
  this->InitialBounds[2] = this->InitialBounds[3] = y;
  this->InitialBounds[4] = this->InitialBounds[5] = z;

  auto* ren = this->GetCurrentRenderer();
  if (!ren) return;

  UpdateWidgetGeometry();

  if (auto* rw = ren->GetRenderWindow())
    rw->Render();
}

//------------------------------------------------------------------------------
void vtkTransformationWidget::PlaceWidget()
{

  double b[6] = {
    this->InitialBounds[0], this->InitialBounds[1],
    this->InitialBounds[2], this->InitialBounds[3],
    this->InitialBounds[4], this->InitialBounds[5]
  };

  const bool haveValidInitial =
    (b[0] < b[1]) && (b[2] < b[3]) && (b[4] < b[5]);

  if (!haveValidInitial)
  {
    b[0] = -1.0; b[1] =  1.0;
    b[2] = -1.0; b[3] =  1.0;
    b[4] = -1.0; b[5] =  1.0;
  }

  this->PlaceWidget(b);
}

        //------------------------------------------------------------------------------
        void vtkTransformationWidget::onLeftButtonDown()
        {
			int mousePos[2];
			this->GetInteractor()->GetEventPosition(mousePos);
			PickHandle(mousePos); 
				
            if (arrowXPicked)
            {
                isAxisDragging = true;
                SetHandlesVisibility(true, true, true, false, false, false, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, true, false, false);
            }      
            if (arrowYPicked)
            {
                isAxisDragging = true;
                SetHandlesVisibility(true, true, true, false, false, false, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, false, true, false);
            }    
            if (arrowZPicked)
            {
                isAxisDragging = true;
                SetHandlesVisibility(true, true, true, false, false, false, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, false, false, true);
            }
            
            if (quadXYPicked)
            {
                isPlaneDragging = true;
                SetHandlesVisibility(true, true, true, true, false, false, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, true, true, false);
            }
              
            if (quadXZPicked)
            {
                isPlaneDragging = true;
                SetHandlesVisibility(true, true, true, false, true, false, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, true, false, true);
            }

            if (quadZYPicked)
            {
                isPlaneDragging = true;
                SetHandlesVisibility(true, true, true, false, false, true, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, false, true, true);
            }
            if (rotXPicked)
            {
                isRotDragging = true;
                SetHandlesVisibility(false, false, false, false, false, false, true, false, false, true);
                SetArcsAndLinesVisibility(true, false, false, true, false, false);
            }
            if (rotYPicked)
            {
                isRotDragging = true;
                SetHandlesVisibility(false, false, false, false, false, false, false, true, false, true);
                SetArcsAndLinesVisibility(false, true, false, false, true, false);
            }
            if (rotZPicked)
            {
                isRotDragging = true;
                SetHandlesVisibility(false, false, false, false, false, false, false, false, true, true);
                SetArcsAndLinesVisibility(false, false, true, false, false, true);
            }
            if (centerPicked)
            {
                isCenterDragging = true;
                SetHandlesVisibility(true, true, true, true, true, true, false, false, false, true);
                SetArcsAndLinesVisibility(false, false, false, false, false, false);
            }
        
            //update start x
            this->GetInteractor()->GetEventPosition(mousePos);
            startMouseX = mousePos[0];
            startMouseY = mousePos[1];
            widgetCSStart = widgetCS;
				
			const bool pickedSomething = (isAxisDragging || isPlaneDragging || isRotDragging || isCenterDragging);
			if (pickedSomething)
			{
				double o[3];
				this->widgetCSStart.GetOrigin(o);

				if (isAxisDragging  || isPlaneDragging  || isCenterDragging)
				{
					
				    this->EndRotationOverlay();
					
					double o[3];
					this->widgetCSStart.GetOrigin(o);
					this->TranslationStartWorld[0] = o[0];
					this->TranslationStartWorld[1] = o[1];
					this->TranslationStartWorld[2] = o[2];	
					this->BeginTranslationOverlay(this->TranslationStartWorld);
					SetOcclusionSphereVisible(false);
				}
				else
				{
					this->EndTranslationOverlay();
				}
				
				if (isRotDragging)
				{
					this->CaptureRotationStartDirWorld();
					this->EndTranslationOverlay();
					this->BeginRotationOverlay(o);
					this->BeginRotationArcOverlay();
					SetOcclusionSphereVisible(false);
				}
				else
				{
					this->EndRotationOverlay();
				}
				
				this->ApplyInteractionColorsForPickedHandle();
				this->EventCallbackCommand->SetAbortFlag(1);
				this->StartInteraction();
				this->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
				this->RefreshInteractor();
			}
		
        }
        //------------------------------------------------------------------------------
        vtkSmartPointer<vtkActor> vtkTransformationWidget::CreateDisc(double radius, int plane, const RGBColorValue color,
            double rotationAngle, const double rotationAxis[3])
        {
            const int numSlices = 100;
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            points->InsertNextPoint(0.0, 0.0, 0.0);

            for (int i = 0; i <= numSlices; i++)
            {
                double angle = vtkMath::RadiansFromDegrees((360.0 * i) / numSlices);
                double x, y, z;
                if (plane == 0)
                {
                    x = 0.0;
                    y = radius * cos(angle);
                    z = radius * sin(angle);
                }
                else if (plane == 1)
                {
                    x = radius * cos(angle);
                    y = 0.0;
                    z = radius * sin(angle);
                }
                else // plane == 2
                {
                    x = radius * cos(angle);
                    y = radius * sin(angle);
                    z = 0.0;
                }
                points->InsertNextPoint(x, y, z);
            }

            vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
            for (int i = 1; i < numSlices; i++)
            {
                vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
                triangle->GetPointIds()->SetId(0, 0);   // Center point.
                triangle->GetPointIds()->SetId(1, i);
                triangle->GetPointIds()->SetId(2, i + 1);
                triangles->InsertNextCell(triangle);
            }
            // Close the circle.
            vtkSmartPointer<vtkTriangle> lastTriangle = vtkSmartPointer<vtkTriangle>::New();
            lastTriangle->GetPointIds()->SetId(0, 0);
            lastTriangle->GetPointIds()->SetId(1, numSlices);
            lastTriangle->GetPointIds()->SetId(2, 1);
            triangles->InsertNextCell(lastTriangle);

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->SetPolys(triangles);

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);

            vtkSmartPointer<vtkActor> arcActor = vtkSmartPointer<vtkActor>::New();
            arcActor->SetMapper(mapper);
			//arcActor->ForceOpaqueOn(); 
            arcActor->GetProperty()->SetColor(color[0], color[1], color[2]);
            arcActor->GetProperty()->SetOpacity(0.5);
            arcActor->SetVisibility(false);

            if (rotationAngle != 0.0)
            {
                vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
                transform->Identity();
                transform->RotateWXYZ(rotationAngle, rotationAxis);
                arcActor->SetUserTransform(transform);
            }
            return arcActor;
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateXDisc(double radius)
        {
            double rotationAxis[3] = { 1.0, 0.0, 0.0 };
            ArcXActor = CreateDisc(radius, 0, X_ARROW_COLOR, 90.0, rotationAxis);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateYDisc(double radius)
        {
            double rotationAxis[3] = { 0.0, 1.0, 0.0 };
            ArcYActor = CreateDisc(radius, 1, Y_ARROW_COLOR, 90.0, rotationAxis);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateZDisc(double radius)
        {
            double rotationAxis[3] = { 0.0, 0.0, 1.0 };
            ArcZActor = CreateDisc(radius, 2, Z_ARROW_COLOR, 90.0, rotationAxis);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::onLeftButtonUp()
        {

		    const bool wasDragging = (isAxisDragging || isPlaneDragging || isRotDragging || isCenterDragging);
			if (!wasDragging)
			{
				return;
			}

			this->EventCallbackCommand->SetAbortFlag(1);
			
			Init();
			EndTranslationOverlay();
			EndRotationOverlay();
			EndRotationArcOverlay();
			SetOcclusionSphereVisible(true);
			
			UpdateWidgetGeometry();
			
			//if (this->GetCurrentRenderer()) {
			//	this->GetCurrentRenderer()->ResetCameraClippingRange();
			//}


			this->EventCallbackCommand->SetAbortFlag(1);
			this->ReleaseFocus();

			this->EndInteraction();
			this->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);

			RefreshInteractor(); // or this->Interactor->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::setVisible(bool visible, bool helper)
        {
            if (visible)
            {
                SetHandlesVisibility(true, true, true, true, true, true, true, true, true, true);
                if(helper)
                    SetArcsAndLinesVisibility(false, false, false, false, false, false);
            }
            else
            {
                SetHandlesVisibility(false, false, false, false, false, false, false, false, false, false);
                SetArcsAndLinesVisibility(false, false, false, false, false, false);
            }
        }
        //------------------------------------------------------------------------------s
        void vtkTransformationWidget::Init()
        {
			ResetState(); 
			
			
            SetHandlesVisibility(true, true, true, true, true, true, true, true, true, true);
            SetArcsAndLinesVisibility(false, false, false, false, false, false);

            // Reset to normal colors
            SetHandleColor(this->ArrowXActor, X_ARROW_COLOR);
            SetHandleColor(this->ArrowYActor, Y_ARROW_COLOR);
            SetHandleColor(this->ArrowZActor, Z_ARROW_COLOR);
            SetHandleColor(this->ArcYActor, Y_ARROW_COLOR);
            SetHandleColor(this->YModeLineActor, Y_ARROW_COLOR);

            isAxisDragging = false;
            isPlaneDragging = false;
            isRotDragging = false;
            isCenterDragging = false;

            quadXYPicked = false;
            quadXZPicked = false;
            quadZYPicked = false;
            arrowXPicked = false;
            arrowYPicked = false;
            arrowZPicked = false;
            centerPicked = false;

            rotXPicked = false;
            rotYPicked = false;
            rotZPicked = false;
        }
        //------------------------------------------------------------------------------
        void  vtkTransformationWidget::ProcessEvents(vtkObject* object, unsigned long event, void* clientdata, void* calldata)
        {
			vtkTransformationWidget* self = static_cast<vtkTransformationWidget*>(clientdata);
			  if (self->m_frontRenderer)
			  {
				self->SetCurrentRenderer(self->m_frontRenderer);
			  }

			switch (event)
			{
				case vtkCommand::MouseMoveEvent:
				{
					self->OnMouseMove(object, event, calldata);
					break;
				}
				case vtkCommand::LeftButtonPressEvent:
				{
					self->onLeftButtonDown();
					break;
				}
				case vtkCommand::LeftButtonReleaseEvent:
				{
					self->onLeftButtonUp();
					break;
				}
				default:
				{
					break;
				}
			}
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::UpdateScaleToCamera(vtkObject* caller, unsigned long, void*)
        {
            UpdateWidgetGeometry();
        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetHandleColor(vtkSmartPointer<vtkActor> actor, const RGBColorValue color)
		{
			if (!actor) { return; }

			vtkProperty* p = actor->GetProperty();
			if (!p) { return; }

			p->SetColor(color[0], color[1], color[2]);
			p->SetAmbientColor(color[0], color[1], color[2]);
			p->SetDiffuseColor(color[0], color[1], color[2]);
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::OnMouseMove(vtkObject* caller, unsigned long eventId, void* callData)
        {
            vtkRenderWindowInteractor* interactor = this->GetInteractor();

            int mousePos[2] = { 0, 0 };
            interactor->GetEventPosition(mousePos);

            if (isAxisDragging)
            {
                moveAlongAxis(mousePos[0], mousePos[1]);
				this->EventCallbackCommand->SetAbortFlag(1);
				this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
				this->RefreshInteractor();
                return;
            }
            if (isPlaneDragging)
            {
                movePlane(mousePos[0], mousePos[1]);
				this->EventCallbackCommand->SetAbortFlag(1);
				this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
				this->RefreshInteractor();
                return;
            }
            if (isRotDragging)
            {
                rotateAxis(mousePos[0], mousePos[1]);
				this->EventCallbackCommand->SetAbortFlag(1);
				this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
				this->RefreshInteractor();
                return;
            }
            if (isCenterDragging)
            {
                moveCenter(mousePos[0], mousePos[1]);
				this->EventCallbackCommand->SetAbortFlag(1);
				this->InvokeEvent(vtkCommand::InteractionEvent, nullptr);
				this->RefreshInteractor();
                return;
            }

            PickHandle(mousePos);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::PickHandle(int  mousePos[2])
        {
            vtkSmartPointer<vtkCellPicker> cellPicker = vtkSmartPointer<vtkCellPicker>::New();
            vtkRenderer* currentRenderer = this->GetCurrentRenderer();
            cellPicker->Pick(mousePos[0], mousePos[1], 0.0, currentRenderer);


            ResetState();

            vtkAssemblyPath* assemblyPath = cellPicker->GetPath();
            if (assemblyPath)
            {
                vtkActor* pickedActor = vtkActor::SafeDownCast(assemblyPath->GetLastNode()->GetViewProp());

                //store picked offset
                double origin[3];
                widgetCS.GetOrigin(origin);

                cellPicker->GetPickPosition(LastPickPosition);
                vtkMath::Subtract(LastPickPosition, origin, m_pickedPointOffset);

                if (pickedActor == this->ArrowXActor)
                {
                    SetHandleColor(this->ArrowXActor, HOVER_COLOR);
                    arrowXPicked = true;
             
                    double axis[3];
                    widgetCS.GetX(axis);

                    double dot = vtkMath::Dot(m_pickedPointOffset, axis);
                    m_pickedPointOffset[0] = dot * axis[0];
                    m_pickedPointOffset[1] = dot * axis[1];
                    m_pickedPointOffset[2] = dot * axis[2];
                }
                else if (pickedActor == this->ArrowYActor)
                {
                    SetHandleColor(this->ArrowYActor, HOVER_COLOR);
                    arrowYPicked = true;
               
                    double axis[3];
                    widgetCS.GetY(axis);

                    double dot = vtkMath::Dot(m_pickedPointOffset, axis);
                    m_pickedPointOffset[0] = dot * axis[0];
                    m_pickedPointOffset[1] = dot * axis[1];
                    m_pickedPointOffset[2] = dot * axis[2];
                }
                else if (pickedActor == this->ArrowZActor)
                {
                    SetHandleColor(this->ArrowZActor, HOVER_COLOR);
                    arrowZPicked = true;
               
                    double axis[3];
                    widgetCS.GetZ(axis);

                    double dot = vtkMath::Dot(m_pickedPointOffset, axis);
                    m_pickedPointOffset[0] = dot * axis[0];
                    m_pickedPointOffset[1] = dot * axis[1];
                    m_pickedPointOffset[2] = dot * axis[2];
                }

                else if (pickedActor == this->QuadXYActor)
                {
                    SetHandleColor(this->QuadXYActor, HOVER_COLOR);
                    quadXYPicked = true;
                }
                else if (pickedActor == this->QuadXZActor)
                {
                    SetHandleColor(this->QuadXZActor, HOVER_COLOR);
                    quadXZPicked = true;
                }
                else if (pickedActor == this->QuadZYActor)
                {
                    SetHandleColor(this->QuadZYActor, HOVER_COLOR);
                    quadZYPicked = true;
                }
                else if (pickedActor == this->RotXActor)
                {
                    SetHandleColor(this->RotXActor, HOVER_COLOR);
                    rotXPicked = true;
                }
                else if (pickedActor == this->RotYActor)
                {
                    SetHandleColor(this->RotYActor, HOVER_COLOR);

                    if (YDiscVisible)
                    {
                        SetHandleColor(this->ArcYActor, Y_HOVER_COLOR);
                        SetHandleColor(this->YModeLineActor, Y_HOVER_COLOR);
                    }
            
                    rotYPicked = true;
                }
                else if (pickedActor == this->RotZActor)
                {
                    SetHandleColor(this->RotZActor, HOVER_COLOR);
                    rotZPicked = true;
                }
                else if (pickedActor == this->CenterSphereActor)
                {
                    SetHandleColor(this->CenterSphereActor, HOVER_COLOR);
                    centerPicked = true;
                }
            }

            currentRenderer->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::ResetState()
        {
            quadXYPicked = false;
            quadXZPicked = false;
            quadZYPicked = false;
            arrowXPicked = false;
            arrowYPicked = false;
            arrowZPicked = false;
            rotXPicked = false;
            rotYPicked = false;
            rotZPicked = false;
            centerPicked = false;

            // Reset colors
            SetHandleColor(this->ArrowXActor, X_ARROW_COLOR);
            SetHandleColor(this->ArrowYActor, Y_ARROW_COLOR);
            SetHandleColor(this->ArrowZActor, Z_ARROW_COLOR);
            SetHandleColor(this->QuadXYActor, Z_ARROW_COLOR);
            SetHandleColor(this->QuadXZActor, Y_ARROW_COLOR);
            SetHandleColor(this->QuadZYActor, X_ARROW_COLOR);
            SetHandleColor(this->RotXActor, X_ARROW_COLOR);
            SetHandleColor(this->RotYActor, Y_ARROW_COLOR);
            SetHandleColor(this->RotZActor, Z_ARROW_COLOR);
            SetHandleColor(this->CenterSphereActor, WHITE_COLOR);
            SetHandleColor(this->ArcYActor, Y_ARROW_COLOR);
            SetHandleColor(this->YModeLineActor, Y_ARROW_COLOR);
			
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::CreateRotZHandle(double radius)
        {
            vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
            polygonSource->SetNumberOfSides(100);
            polygonSource->SetRadius(radius);
            polygonSource->SetNormal(0.0, 0.0, 1.0);
            polygonSource->SetCenter(0.0, 0.0, 0.0);

            vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
            tubeFilter->SetInputConnection(polygonSource->GetOutputPort());
            tubeFilter->SetRadius(0.01);
            tubeFilter->SetNumberOfSides(12);
            tubeFilter->CappingOn();
            tubeFilter->Update();

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(tubeFilter->GetOutputPort());

            RotZActor = vtkSmartPointer<vtkActor>::New();
            RotZActor->SetMapper(mapper);
            RotZActor->GetProperty()->SetColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            RotZActor->SetVisibility(RotZActive);
            RotZActor->GetProperty()->SetAmbientColor(Z_ARROW_COLOR[0], Z_ARROW_COLOR[1], Z_ARROW_COLOR[2]);
            RotZActor->GetProperty()->SetAmbient(1.0);
            RotZActor->GetProperty()->SetDiffuse(0.0);
            RotZActor->GetProperty()->SetSpecular(0.0);
        }
        //--------------CreateRotXHandle----------------------------------------------------------------
        void vtkTransformationWidget::CreateRotXHandle(double radius)
        {
            vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
            polygonSource->SetNumberOfSides(CompassRotPolygonSides); 
            polygonSource->SetRadius(radius);
            polygonSource->SetNormal(1.0, 0.0, 0.0);
            polygonSource->SetCenter(0.0, 0.0, 0.0);

            vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
            tubeFilter->SetInputConnection(polygonSource->GetOutputPort());
            tubeFilter->SetRadius(0.01);
            tubeFilter->SetNumberOfSides(CompassRotTubeSides);
            tubeFilter->CappingOn();
            tubeFilter->Update();

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(tubeFilter->GetOutputPort());

            RotXActor = vtkSmartPointer<vtkActor>::New();
            RotXActor->SetMapper(mapper);
            RotXActor->GetProperty()->SetColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
            RotXActor->SetVisibility(RotXActive);
            RotXActor->GetProperty()->SetAmbientColor(X_ARROW_COLOR[0], X_ARROW_COLOR[1], X_ARROW_COLOR[2]);
            RotXActor->GetProperty()->SetAmbient(1.0);
            RotXActor->GetProperty()->SetDiffuse(0.0);
            RotXActor->GetProperty()->SetSpecular(0.0);
        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::CreateRotYHandle(double radius)
		{
			vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
			polygonSource->SetNumberOfSides(this->CompassRotPolygonSides);
			polygonSource->SetRadius(radius);
			polygonSource->SetNormal(0.0, 1.0, 0.0);
			polygonSource->SetCenter(0.0, 0.0, 0.0);

            vtkSmartPointer<vtkTubeFilter> tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
            tubeFilter->SetInputConnection(polygonSource->GetOutputPort());
            tubeFilter->SetRadius(0.01);
            tubeFilter->SetNumberOfSides(CompassRotTubeSides);
            tubeFilter->CappingOn();
            tubeFilter->Update();

            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputConnection(tubeFilter->GetOutputPort());

            RotYActor = vtkSmartPointer<vtkActor>::New();
            RotYActor->SetMapper(mapper);
            RotYActor->GetProperty()->SetColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]);
            RotYActor->SetVisibility(RotXActive);
            RotYActor->GetProperty()->SetAmbientColor(Y_ARROW_COLOR[0], Y_ARROW_COLOR[1], Y_ARROW_COLOR[2]);
            RotYActor->GetProperty()->SetAmbient(1.0);
            RotYActor->GetProperty()->SetDiffuse(0.0);
            RotYActor->GetProperty()->SetSpecular(0.0);
		}
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::rotateAxis(int x, int y)
	{
		double axisW[3] = { 0.0, 0.0, 1.0 };
		if (rotXPicked) { widgetCSStart.GetX(axisW); }
		else if (rotYPicked) { widgetCSStart.GetY(axisW); }
		else { widgetCSStart.GetZ(axisW); }

		if (vtkMath::Normalize(axisW) == 0.0)
		{
			return;
		}

		double centerW[3];
		widgetCSStart.GetOrigin(centerW);

		// Ray from mouse
		double p0w4[4], p1w4[4];
		this->ComputeDisplayToWorld(double(x), double(y), 0.0, p0w4);
		this->ComputeDisplayToWorld(double(x), double(y), 1.0, p1w4);
		vtkMath::Subtract(p1w4, p0w4, p1w4);

		double p0w[3]  = { p0w4[0], p0w4[1], p0w4[2] };
		double dirW[3] = { p1w4[0], p1w4[1], p1w4[2] };

		// Intersect with rotation plane: (centerW, normal=axisW)
		double hitW[3];
		const double offset = vtkMath::Dot(centerW, axisW);
		if (!this->rayPlaneIntersect(p0w, dirW, axisW, offset, hitW))
		{
			return;
		}

		// Project hit onto disc plane in WORLD coords (remove axis component)
		double curDirW[3] = {
			hitW[0] - centerW[0],
			hitW[1] - centerW[1],
			hitW[2] - centerW[2]
		};

		const double dW = vtkMath::Dot(curDirW, axisW);
		curDirW[0] -= dW * axisW[0];
		curDirW[1] -= dW * axisW[1];
		curDirW[2] -= dW * axisW[2];

		const double radiusW = vtkMath::Normalize(curDirW); // normalizes + returns length
		if (radiusW <= 1e-12)
		{
			return;
		}

		// Signed angle on the disc (WORLD): atan2( axis · (start x cur), start · cur )
		double crossSC[3];
		vtkMath::Cross(this->RotStartDirWorld, curDirW, crossSC);

		const double sinTerm = vtkMath::Dot(axisW, crossSC);
		const double cosTerm = vtkMath::ClampValue(
			vtkMath::Dot(this->RotStartDirWorld, curDirW), -1.0, 1.0);

		const double thetaRad = std::atan2(sinTerm, cosTerm);
		double thetaDeg = vtkMath::DegreesFromRadians(thetaRad);

		const bool freeRotate = (this->GetInteractor() && this->GetInteractor()->GetShiftKey() != 0);
		const double snapStepDeg = 5.0;

		double thetaUsedDeg = thetaDeg;
		if (!freeRotate)
		{
			thetaUsedDeg = std::round(thetaDeg / snapStepDeg) * snapStepDeg;
		}

		m_rotationTheta = thetaUsedDeg;

		vtkSmartPointer<vtkTransform> incremental = vtkSmartPointer<vtkTransform>::New();
		incremental->Identity();
		incremental->Translate(centerW[0], centerW[1], centerW[2]);
		incremental->RotateWXYZ(thetaUsedDeg, axisW);
		incremental->Translate(-centerW[0], -centerW[1], -centerW[2]);

		{
			double dir[3];

			widgetCSStart.GetX(dir);
			incremental->TransformVector(dir, dir);
			vtkMath::Normalize(dir);
			widgetCS.SetDirX(dir[0], dir[1], dir[2]);

			widgetCSStart.GetY(dir);
			incremental->TransformVector(dir, dir);
			vtkMath::Normalize(dir);
			widgetCS.SetDirY(dir[0], dir[1], dir[2]);

			widgetCSStart.GetZ(dir);
			incremental->TransformVector(dir, dir);
			vtkMath::Normalize(dir);
			widgetCS.SetDirZ(dir[0], dir[1], dir[2]);
		}

		// --- Overlays: use snapped hit point when snapping ---
		double hitForOverlayW[3] = { hitW[0], hitW[1], hitW[2] };

		if (!freeRotate)
		{
			vtkNew<vtkTransform> rot;
			rot->Identity();
			rot->RotateWXYZ(thetaUsedDeg, axisW);

			double snappedDirW[3] = {
				this->RotStartDirWorld[0],
				this->RotStartDirWorld[1],
				this->RotStartDirWorld[2]
			};
			rot->TransformVector(snappedDirW, snappedDirW);
			vtkMath::Normalize(snappedDirW);

			hitForOverlayW[0] = centerW[0] + snappedDirW[0] * radiusW;
			hitForOverlayW[1] = centerW[1] + snappedDirW[1] * radiusW;
			hitForOverlayW[2] = centerW[2] + snappedDirW[2] * radiusW;
		}

		this->UpdateRotationOverlay(thetaUsedDeg, centerW, hitForOverlayW);
		this->UpdateRotationArcOverlayFromHitWorld(hitForOverlayW);

		UpdateWidgetGeometry();
		PropagateTransformModifiedEvent();
	}

       //------------------------------------------------------------------------------
        void vtkTransformationWidget::moveAlongAxis(int x, int y)
        {
            vtkRenderer* renderer = this->GetCurrentRenderer();
            if (!renderer) return;
            vtkCamera* camera = renderer->GetActiveCamera();
            if (!camera) return;

            // Compute mouse movement delta.
            int deltaX = x - startMouseX;
            int deltaY = y - startMouseY;
            if (std::abs(deltaX) < 2 && std::abs(deltaY) < 2)
                return;

            // Get the current origin 
            double origin[3];
            widgetCS.GetOrigin(origin);

            // Get the direction 
            double dirManip[3] = { 0.0, 0.0, 0.0 };
            if (arrowXPicked)
                widgetCS.GetX(dirManip);
            else if (arrowYPicked)
                widgetCS.GetY(dirManip);
            else if (arrowZPicked)
                widgetCS.GetZ(dirManip);

            // Compute the ray through the current display point.
            double pRay[4], dirRay[4];
            ComputeDisplayToWorld(double(x), double(y), 0, pRay);
            ComputeDisplayToWorld(double(x), double(y), 1, dirRay);
            vtkMath::Subtract(dirRay, pRay, dirRay);

            // Compute the closest point along the manipulation axis.
            double dirManipRay[3];
            vtkMath::Subtract(pRay, origin, dirManipRay);
            double dirCross[3];
            vtkMath::Cross(dirRay, dirManip, dirCross);
            double dirCrossSquareLength = vtkMath::Dot(dirCross, dirCross);
            if (dirCrossSquareLength == 0.0)
                return;  // The lines are parallel, no movement.

            double R[3];
            vtkMath::Cross(dirManipRay, dirCross, R);
            vtkMath::MultiplyScalar(R, 1.0 / dirCrossSquareLength);
            double t1 = vtkMath::Dot(R, dirRay);

            // Compute new origin along the axis.
            double newOrigin[3];
            {
                // Multiply the manipulation direction by the computed scalar.
                double scaledDir[3] = { dirManip[0] * t1, dirManip[1] * t1, dirManip[2] * t1 };
                vtkMath::Add(origin, scaledDir, newOrigin);
            }

            // picking offset
            double offsettedNewOrigin[3];
            vtkMath::Subtract(newOrigin, m_pickedPointOffset, offsettedNewOrigin);
            widgetCS.SetOrigin(offsettedNewOrigin[0], offsettedNewOrigin[1], offsettedNewOrigin[2]);

            UpdateWidgetGeometry();
            PropagateTransformModifiedEvent();
			
			double cur[3];
			widgetCS.GetOrigin(cur);
			UpdateTranslationOverlay(cur);
        }

        void vtkTransformationWidget::moveCenter(int x, int y)
        {
            vtkRenderer* renderer = this->GetCurrentRenderer();
            if (!renderer) return;
            vtkCamera* camera = renderer->GetActiveCamera();
            if (!camera) return;

            double origin[3];
            widgetCS.GetOrigin(origin);

            double viewNormal[3];
            camera->GetViewPlaneNormal(viewNormal);

            double planeOffset = vtkMath::Dot(origin, viewNormal);

            double pRay[4], dirRay[4];
            ComputeDisplayToWorld(static_cast<double>(x), static_cast<double>(y), 0, pRay);
            ComputeDisplayToWorld(static_cast<double>(x), static_cast<double>(y), 1, dirRay);
            vtkMath::Subtract(dirRay, pRay, dirRay);

            double hitPoint[3];
            if (rayPlaneIntersect(pRay, dirRay, viewNormal, planeOffset, hitPoint))
            {
                double newCenter[3];
                vtkMath::Subtract(hitPoint, m_pickedPointOffset, newCenter);
                widgetCS.SetOrigin(newCenter[0], newCenter[1], newCenter[2]);

                UpdateWidgetGeometry();
                PropagateTransformModifiedEvent();
            }
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::movePlane(int x, int y)
        {
            double origin[3];
             widgetCS.GetOrigin(origin); 

            double normal[3] = { 0.0, 0.0, 0.0 };
            if (quadXYPicked) 
                widgetCS.GetZ(normal);
            else if (quadZYPicked) 
                widgetCS.GetX(normal);
            else if (quadXZPicked) 
                widgetCS.GetY(normal);
            
            // The ray through the picked pixel
            double pRay[4];
            ComputeDisplayToWorld(double(x), double(y), 0, pRay); 
            double dirRay[4];
            ComputeDisplayToWorld(double(x), double(y), 1, dirRay); 
            vtkMath::Subtract(dirRay, pRay, dirRay); 

            double hitPoint[3];

            // Perform ray-plane intersection
            if (rayPlaneIntersect(pRay, dirRay, normal, vtkMath::Dot(origin, normal), hitPoint)) 
            {
                vtkMath::Subtract(hitPoint, m_pickedPointOffset, hitPoint);
                widgetCS.SetOrigin(hitPoint[0], hitPoint[1], hitPoint[2]);
                
                UpdateWidgetGeometry();
                PropagateTransformModifiedEvent();
				
				double cur[3];
				widgetCS.GetOrigin(cur);
				UpdateTranslationOverlay(cur);
            }
        }
        //------------------------------------------------------------------------------
        bool vtkTransformationWidget::rayPlaneIntersect(const double origin[3], const double direction[3], const double n[3], 
            const double offset, double hitPoint[3])
        {
            const double epsilon = 100 * std::numeric_limits<double>::epsilon();

            // check if ray is parallel to plane
            double denominator = vtkMath::Dot(n, direction);
            if (denominator > -epsilon && denominator < epsilon)
                return false;

            // calculate the distance from the rays origin to intersection
            double nDotOrigin = vtkMath::Dot(n, origin);
            double parameter = (-nDotOrigin + offset) / denominator;

            vtkScale(direction, parameter, hitPoint);
            vtkAdd(hitPoint, origin, hitPoint);

            return true;
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::vtkScale(const double a[3], const double b, double c[3])
        {
            c[0] = a[0] * b;
            c[1] = a[1] * b;
            c[2] = a[2] * b;
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::vtkDivide(const double a[3], const double b, double c[3])
        {
            c[0] = a[0] / b;
            c[1] = a[1] / b;
            c[2] = a[2] / b;
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::vtkAdd(const double a[3], const double b[3], double c[3])
        {
            c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            c[2] = a[2] + b[2];
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::UpdateWidgetGeometry()
        {      
            double scaleFactor = ComputeScalingFactor(this->CompassDesiredPixelSizePx);

            double origin[3], dirX[3], dirY[3], dirZ[3];
            widgetCS.GetOrigin(origin);
            widgetCS.GetX(dirX);
            widgetCS.GetY(dirY);
            widgetCS.GetZ(dirZ);

            vtkSmartPointer<vtkMatrix4x4> transformMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
            for (int i = 0; i < 3; i++)
            {
                transformMatrix->SetElement(i, 0, scaleFactor * dirX[i]);
                transformMatrix->SetElement(i, 1, scaleFactor * dirY[i]);
                transformMatrix->SetElement(i, 2, scaleFactor * dirZ[i]);
                transformMatrix->SetElement(i, 3, origin[i]);
            }

            transformMatrix->SetElement(3, 0, 0);
            transformMatrix->SetElement(3, 1, 0);
            transformMatrix->SetElement(3, 2, 0);
            transformMatrix->SetElement(3, 3, 1);

            vtkSmartPointer<vtkTransform> newTransform = vtkSmartPointer<vtkTransform>::New();
            newTransform->SetMatrix(transformMatrix);
            this->AxesAssembly->SetUserTransform(newTransform);

            double updatedOrigin[3] = {
                transformMatrix->GetElement(0, 3),
                transformMatrix->GetElement(1, 3),
                transformMatrix->GetElement(2, 3)
            };
            widgetCS.SetOrigin(updatedOrigin[0], updatedOrigin[1], updatedOrigin[2]);
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::PropagateTransformModifiedEvent()
        {
            double origin[3], dirX[3], dirY[3], dirZ[3];
            widgetCS.GetOrigin(origin);
            widgetCS.GetX(dirX);
            widgetCS.GetY(dirY);
            widgetCS.GetZ(dirZ);

            vtkSmartPointer<vtkMatrix4x4> transformMatrix2 = vtkSmartPointer<vtkMatrix4x4>::New();
            for (int i = 0; i < 3; i++)
            {
                transformMatrix2->SetElement(i, 0, dirX[i]);
                transformMatrix2->SetElement(i, 1, dirY[i]);
                transformMatrix2->SetElement(i, 2, dirZ[i]);
                transformMatrix2->SetElement(i, 3, origin[i]);
            }
            transformMatrix2->SetElement(3, 0, 0);
            transformMatrix2->SetElement(3, 1, 0);
            transformMatrix2->SetElement(3, 2, 0);
            transformMatrix2->SetElement(3, 3, 1);

            vtkSmartPointer<vtkTransform> newTransform2 = vtkSmartPointer<vtkTransform>::New();
            newTransform2->SetMatrix(transformMatrix2);
            this->InvokeEvent(TransformationWidgetTransformModifiedEvent, newTransform2);
        }
        //------------------------------------------------------------------------------
		double vtkTransformationWidget::ComputeScalingFactor(double desiredPixelSize)
		{
			vtkRenderer* renderer = this->GetCurrentRenderer();
			if (!renderer)
			{
				return 1.0;
			}

			vtkCamera* camera = renderer->GetActiveCamera();
			if (!camera)
			{
				return 1.0;
			}

			int* vpSize = renderer->GetSize(); // viewport size in pixels
			if (!vpSize || vpSize[1] <= 0)
			{
				return 1.0;
			}

			double viewHeightWorld = 1.0;

			if (camera->GetParallelProjection())
			{
				// Ortho: ParallelScale is half the viewport height in world units
				viewHeightWorld = 2.0 * camera->GetParallelScale();
			}
			else
			{
				// Perspective: viewport height at the widget depth
				double origin[3];
				this->widgetCS.GetOrigin(origin);

				double camPos[3];
				camera->GetPosition(camPos);

				double dop[3];
				camera->GetDirectionOfProjection(dop);
				if (vtkMath::Normalize(dop) == 0.0)
				{
					return 1.0;
				}

				double v[3] = { origin[0] - camPos[0], origin[1] - camPos[1], origin[2] - camPos[2] };
				double dist = vtkMath::Dot(v, dop); // depth along view direction
				if (dist <= 1e-9)
				{
					dist = camera->GetDistance();
					if (dist <= 1e-9)
					{
						return 1.0;
					}
				}

				const double fovRadians = vtkMath::RadiansFromDegrees(camera->GetViewAngle());
				viewHeightWorld = 2.0 * dist * std::tan(0.5 * fovRadians);
			}

			const double worldPerPixel = viewHeightWorld / static_cast<double>(vpSize[1]);
			return desiredPixelSize * worldPerPixel;
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetArrowXHandleActive(bool active)
        {
            ArrowXActive = active;
            if (this->ArrowXActor)
                this->ArrowXActor->SetVisibility(active);
            if (this->ArrowXLineActor)
                this->ArrowXLineActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::RotateY(const float angle)
        {
            double center[3];
            widgetCS.GetOrigin(center);

            vtkSmartPointer<vtkTransform> rotationTransform = vtkSmartPointer<vtkTransform>::New();
            rotationTransform->Identity();
            rotationTransform->Translate(center[0], center[1], center[2]);
            rotationTransform->RotateWXYZ(angle, 0.0, 1.0, 0.0);
            rotationTransform->Translate(-center[0], -center[1], -center[2]);

            double xAxis[3], yAxis[3], zAxis[3];
            widgetCS.GetX(xAxis);
            widgetCS.GetY(yAxis);
            widgetCS.GetZ(zAxis);

            rotationTransform->TransformVector(xAxis, xAxis);
            rotationTransform->TransformVector(yAxis, yAxis);
            rotationTransform->TransformVector(zAxis, zAxis);

            vtkMath::Normalize(xAxis);
            vtkMath::Normalize(yAxis);
            vtkMath::Normalize(zAxis);

            widgetCS.SetDirX(xAxis[0], xAxis[1], xAxis[2]);
            widgetCS.SetDirY(yAxis[0], yAxis[1], yAxis[2]);
            widgetCS.SetDirZ(zAxis[0], zAxis[1], zAxis[2]);

            this->UpdateWidgetGeometry();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetRotationModeY()
        {			
            SetYDiscVisible(true);
            SetCenterHandleActive(false);
            SetArrowXHandleActive(true);
            SetArrowYHandleActive(false);
            SetArrowZHandleActive(false);
            SetRotXHandleActive(false);
            SetRotYHandleActive(true);
            SetRotZHandleActive(false);
            SetQuadXYHandleActive(false);
            SetQuadXZHandleActive(false);
            SetQuadZYHandleActive(false);
        }
		//------------------------------------------------------------------------------		
		void vtkTransformationWidget::SetZDiscVisible(bool active)
		{
			this->ZDiscVisible = active;
		}
		 //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetRotationModeZ()
		{
			if (!this->AxesAssembly || !this->ArrowZLineActor || !this->RotZActor || !this->ArcZActor) {
				this->CreateDefaultRepresentation();
			}

			this->translationMode = false;

			// Rotation handles: only Z
			this->SetRotXHandleActive(false);
			this->SetRotYHandleActive(false);
			this->SetRotZHandleActive(true);

			// Disc visible
			this->SetZDiscVisible(true);

			// Translation arrows OFF (not pickable/visible)
			this->SetArrowXHandleActive(true);
			this->SetArrowYHandleActive(true);
			this->SetArrowZHandleActive(false);

			// Planes OFF
			this->SetQuadXYHandleActive(true);
			this->SetQuadXZHandleActive(false);
			this->SetQuadZYHandleActive(false);

			// Center OFF
			this->SetCenterHandleActive(false);

			// Axis lines: only Z visible
			this->SetAxisLineXVisible(true);
			this->SetAxisLineYVisible(true);
			this->SetAxisLineZVisible(true);

			// Occlusion sphere ON for idle
			this->SetOcclusionSphereVisible(true);

			this->RefreshInteractor();
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetTranslationMode()
        {
			this->translationMode = true;
			SetOcclusionSphereVisible(false);
			
           SetArrowXHandleActive(true);
           SetArrowYHandleActive(true);
           SetArrowZHandleActive(true);
           SetRotXHandleActive(false);
           SetRotYHandleActive(false);
           SetRotZHandleActive(false);
		   
		  if (this->ArrowXLineActor)this->ArrowXLineActor->SetVisibility(0);
		  if (this->ArrowYLineActor)this->ArrowYLineActor->SetVisibility(0);
		  if (this->ArrowZLineActor)this->ArrowZLineActor->SetVisibility(0);
			
        }
		//------------------------------------------------------------------------------
		void vtkTransformationWidget::SetOcclusionSphereVisible(bool active)
		{
			if (!this->AxesAssembly)
			{
				return;
			}
			
			if(translationMode)
				active = false;
		
			if (active)
			{
				if (this->OcclusionSphereDepthActor)
				{
					//this->AxesAssembly->AddPart(this->OcclusionSphereDepthActor);
					this->OcclusionSphereDepthActor->SetVisibility(1);
				}
				if (this->OcclusionSphereActor)
				{
					//this->AxesAssembly->AddPart(this->OcclusionSphereActor);
					this->OcclusionSphereActor->SetVisibility(1);
				}
			}
			else
			{
				if (this->OcclusionSphereDepthActor)
				{
					this->OcclusionSphereDepthActor->SetVisibility(0);
					//this->AxesAssembly->RemovePart(this->OcclusionSphereDepthActor);	
				}
				if (this->OcclusionSphereActor)
				{
					this->OcclusionSphereActor->SetVisibility(0);
					//this->AxesAssembly->RemovePart(this->OcclusionSphereActor);
				}
			}
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetCenterHandleActive(bool active)
        {
            CenterActive = active;
            if (this->CenterActive)
                this->CenterSphereActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetAxisLineXVisible(bool visible)
		{
			this->AxisLineXVisible = visible;
			if (this->ArrowXLineActor) {
				this->ArrowXLineActor->SetVisibility(visible ? 1 : 0);
			}
			this->RefreshInteractor();
		}
        //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetAxisLineYVisible(bool visible)
		{
			this->AxisLineYVisible = visible;
			if (this->ArrowYLineActor) {
				this->ArrowYLineActor->SetVisibility(visible ? 1 : 0);
			}
			this->RefreshInteractor();
		}
      //------------------------------------------------------------------------------
		void vtkTransformationWidget::SetAxisLineZVisible(bool visible)
		{
			this->AxisLineZVisible = visible;
			if (this->ArrowZLineActor) {
				this->ArrowZLineActor->SetVisibility(visible ? 1 : 0);
			}
			this->RefreshInteractor();
		}
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetArrowYHandleActive(bool active)
        {
            ArrowYActive = active;
            if (this->ArrowYActor)
                this->ArrowYActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetArrowZHandleActive(bool active)
        {
            ArrowZActive = active;
            if (this->ArrowZActor)
                this->ArrowZActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetRotXHandleActive(bool active)
        {
            RotXActive = active;
            if (this->RotXActor)
                this->RotXActor->SetVisibility(active);
            if (this->ArcXActor)
                this->ArcXActor->SetVisibility(active);

            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetRotYHandleActive(bool active)
        {
            RotYActive = active;
            if (this->RotYActor)
                this->RotYActor->SetVisibility(active);
            if (this->ArcYActor)
                this->ArcYActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetRotZHandleActive(bool active)
        {
            RotZActive = active;
            if (this->RotZActor)
                this->RotZActor->SetVisibility(active);
            if (this->ArcZActor)
                this->ArcZActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetQuadXYHandleActive(bool active)
        {
            QuadXYActive = active;
            if (this->QuadXYActor)
                this->QuadXYActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetQuadXZHandleActive(bool active)
        {
            QuadXZActive = active;
            if (this->QuadXZActor)
                this->QuadXZActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetQuadZYHandleActive(bool active)
        {
            QuadZYActive = active;
            if (this->QuadZYActor)
                this->QuadZYActor->SetVisibility(active);
            if (this->GetCurrentRenderer())
                this->GetCurrentRenderer()->GetRenderWindow()->Render();
        }
        //------------------------------------------------------------------------------
        void vtkTransformationWidget::SetYDiscVisible(bool active)
        {
            YDiscVisible = active;
        }

       //------------------------------------------------------------------------------
void vtkTransformationWidget::GetTransform(vtkTransform* t) const
{
  if (!t) return;
  // Build a transform from the current widget coordinate system
  vtkSmartPointer<vtkTransform> csT = this->widgetCS.AsTransform();
  t->SetMatrix(csT->GetMatrix());
}

void vtkTransformationWidget::SetTransform(vtkTransform* t)
{
  if (!t) return;

  vtkMatrix4x4* M = t->GetMatrix();
  if (!M) return;  // hard guard

  double x[3] = { M->GetElement(0,0), M->GetElement(1,0), M->GetElement(2,0) };
  double y[3] = { M->GetElement(0,1), M->GetElement(1,1), M->GetElement(2,1) };
  double z[3] = { M->GetElement(0,2), M->GetElement(1,2), M->GetElement(2,2) };
  double o[3] = { M->GetElement(0,3), M->GetElement(1,3), M->GetElement(2,3) };

  if (vtkMath::Normalize(x) == 0.0) { x[0]=1; x[1]=0; x[2]=0; }
  if (vtkMath::Normalize(y) == 0.0) { y[0]=0; y[1]=1; y[2]=0; }
  if (vtkMath::Normalize(z) == 0.0) { z[0]=0; z[1]=0; z[2]=1; }

  this->widgetCS.SetOrigin(o[0], o[1], o[2]);
  this->widgetCS.SetDirX(x[0], x[1], x[2]);
  this->widgetCS.SetDirY(y[0], y[1], y[2]);
  this->widgetCS.SetDirZ(z[0], z[1], z[2]);

  if (this->GetCurrentRenderer())
  {
    this->UpdateWidgetGeometry();
    this->PropagateTransformModifiedEvent();
  }
}

void vtkTransformationWidget::GetCenter(double center[3]) const
{
   center[0] = 0.0;
   center[1] = 0.0;
   center[2] = 0.0;

   if (!this->AxesAssembly) {
     return;
   }

   vtkLinearTransform* lt = this->AxesAssembly->GetUserTransform();
   vtkMatrix4x4* M = lt ? lt->GetMatrix() : nullptr;
   if (!M) {
     return;
   }

   // World center is the translation column of the assembly transform
   center[0] = M->GetElement(0, 3);
   center[1] = M->GetElement(1, 3);
   center[2] = M->GetElement(2, 3);
}

void vtkTransformationWidget::GetCenter(double& cx, double& cy, double& cz) const
{
   double c[3];
   this->GetCenter(c);
   cx = c[0];
   cy = c[1];
   cz = c[2];
}

void vtkTransformationWidget::RefreshInteractor()
{
  if (auto ren = this->GetCurrentRenderer())
  {
    if (auto rw = ren->GetRenderWindow())
    {
      rw->Render();
    }
  }
}

void vtkTransformationWidget::CreateTranslationOverlayIfNeeded()
{
    vtkRenderer* ren = this->GetCurrentRenderer();
    if (this->TranslationLineSource && this->TranslationLineActor &&
        this->TranslationTextActor && this->TranslationStartMarkerSource &&
        this->TranslationStartMarkerActor)
    {
        return;
    }


    double o[3];
    this->widgetCS.GetOrigin(o);

    // Line
    this->TranslationLineSource = vtkSmartPointer<vtkLineSource>::New();
    this->TranslationLineSource->SetPoint1(o[0], o[1], o[2]);
    this->TranslationLineSource->SetPoint2(o[0], o[1], o[2]);

    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    lineMapper->SetInputConnection(this->TranslationLineSource->GetOutputPort());

    this->TranslationLineActor = vtkSmartPointer<vtkActor>::New();
    this->TranslationLineActor->SetMapper(lineMapper);
    this->TranslationLineActor->PickableOff();
    this->TranslationLineActor->GetProperty()->SetLineWidth(3.0);
    //this->TranslationLineActor->GetProperty()->SetRenderLinesAsTubes(true);
    this->TranslationLineActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    this->TranslationLineActor->SetVisibility(false);

    // Billboard text
    this->TranslationTextActor = vtkSmartPointer<vtkBillboardTextActor3D>::New();
    this->TranslationTextActor->PickableOff();
    this->TranslationTextActor->SetInput("");
    this->TranslationTextActor->SetPosition(o[0], o[1], o[2]);
    this->TranslationTextActor->SetVisibility(false);

    if (vtkTextProperty* tp = this->TranslationTextActor->GetTextProperty())
    {
        tp->SetColor(1.0, 1.0, 1.0);
        tp->SetBackgroundColor(0.0, 0.0, 0.0);
        tp->SetBackgroundOpacity(0.25);
        tp->SetBold(1);
    }

    // Start sphere marker
    this->TranslationStartMarkerSource = vtkSmartPointer<vtkSphereSource>::New();
    this->TranslationStartMarkerSource->SetThetaResolution(16);
    this->TranslationStartMarkerSource->SetPhiResolution(16);
    this->TranslationStartMarkerSource->SetCenter(o[0], o[1], o[2]);

    this->TranslationStartMarkerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    this->TranslationStartMarkerMapper->SetInputConnection(this->TranslationStartMarkerSource->GetOutputPort());

    this->TranslationStartMarkerActor = vtkSmartPointer<vtkActor>::New();
    this->TranslationStartMarkerActor->SetMapper(this->TranslationStartMarkerMapper);
    this->TranslationStartMarkerActor->PickableOff();
    this->TranslationStartMarkerActor->SetVisibility(false);

    this->TranslationStartMarkerActor->GetProperty()->SetColor(1.0, 1.0, 0.0); // yellow
    this->TranslationStartMarkerActor->GetProperty()->SetAmbient(1.0);
    this->TranslationStartMarkerActor->GetProperty()->SetDiffuse(0.0);
    this->TranslationStartMarkerActor->GetProperty()->SetSpecular(0.0);
}

void vtkTransformationWidget::SetFrontRenderer(vtkRenderer* renderer)
{
  m_frontRenderer = renderer;
  if (renderer)
  {
	  
	if (this->m_frontRenderer && this->m_frontRenderer->GetActiveCamera())
    {
        // "Maximal" clipping range
        //this->m_frontRenderer->GetActiveCamera()->SetClippingRange(0.001, 1e9);
    }
	  
    this->SetCurrentRenderer(renderer);	
  }
}

void vtkTransformationWidget::SetSceneRenderer(vtkRenderer* renderer)
{
  m_sceneRenderer = renderer;
}

void vtkTransformationWidget::BeginTranslationOverlay(const double startWorld[3])
{
    this->CreateTranslationOverlayIfNeeded();

    this->TranslationStartWorld[0] = startWorld[0];
    this->TranslationStartWorld[1] = startWorld[1];
    this->TranslationStartWorld[2] = startWorld[2];
	
    if (this->TranslationStartMarkerSource)
    {
        this->TranslationStartMarkerSource->SetCenter(startWorld[0], startWorld[1], startWorld[2]);

        const double markerDiameterWorld = this->ComputeScalingFactor(12.0); // ~12 px diameter
        this->TranslationStartMarkerSource->SetRadius(0.5 * markerDiameterWorld);

        this->TranslationStartMarkerSource->Modified();
    }

    this->TranslationLineSource->SetPoint1(startWorld[0], startWorld[1], startWorld[2]);
    this->TranslationLineSource->SetPoint2(startWorld[0], startWorld[1], startWorld[2]);
    this->TranslationLineSource->Modified();

    if (this->TranslationLineActor) { this->TranslationLineActor->SetVisibility(true); }
    if (this->TranslationStartMarkerActor) { this->TranslationStartMarkerActor->SetVisibility(true); }
    if (this->TranslationTextActor) { this->TranslationTextActor->SetVisibility(true); }

    this->TranslationOverlayActive = true;
    this->RefreshInteractor();
}


void vtkTransformationWidget::EndTranslationOverlay()
{
    this->TranslationOverlayActive = false;
	
    if (this->TranslationLineActor)
    {
        this->TranslationLineActor->SetVisibility(false);
    }
    if (this->TranslationTextActor)
    {
        this->TranslationTextActor->SetVisibility(false);
        this->TranslationTextActor->SetInput("");
    }
    if (this->TranslationStartMarkerActor)
    {
        this->TranslationStartMarkerActor->SetVisibility(false);
    }

    this->RefreshInteractor();
}

static double Clamp01(double v)
{
    if (v < 0.0) { return 0.0; }
    if (v > 1.0) { return 1.0; }
    return v;
}

void vtkTransformationWidget::UpdateTranslationOverlay(const double currentWorld[3])
{
    if (!this->TranslationOverlayActive)
    {
        return;
    }
    if (!this->TranslationLineSource || !this->TranslationTextActor)
    {
        return;
    }

    double endWorld[3] = { currentWorld[0], currentWorld[1], currentWorld[2] };
    const double dx = endWorld[0] - this->TranslationStartWorld[0];
    const double dy = endWorld[1] - this->TranslationStartWorld[1];
    const double dz = endWorld[2] - this->TranslationStartWorld[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Z-fighting mitigation: nudge line + text a tiny bit toward the camera
    double dop[3] = { 0.0, 0.0, 1.0 };
    vtkRenderer* ren = this->GetCurrentRenderer();
    if (ren && ren->GetActiveCamera())
    {
        ren->GetActiveCamera()->GetDirectionOfProjection(dop);
        vtkMath::Normalize(dop);
    }

    double eps = (dist > 1e-9) ? (dist * 1e-3) : 1e-3;

    if (ren && ren->GetActiveCamera())
    {
        double camPos[3];
        ren->GetActiveCamera()->GetPosition(camPos);
        const double camDist = std::sqrt(vtkMath::Distance2BetweenPoints(camPos, this->TranslationStartWorld));
        const double camEps = camDist * 1e-6;
        if (camEps > eps)
        {
            eps = camEps;
        }
    }

    double p1[3] = {
        this->TranslationStartWorld[0] - dop[0] * eps,
        this->TranslationStartWorld[1] - dop[1] * eps,
        this->TranslationStartWorld[2] - dop[2] * eps
    };

    double p2[3] = {
        endWorld[0] - dop[0] * eps,
        endWorld[1] - dop[1] * eps,
        endWorld[2] - dop[2] * eps
    };

    // Update line endpoints
    this->TranslationLineSource->SetPoint1(p1);
    this->TranslationLineSource->SetPoint2(p2);
    this->TranslationLineSource->Modified(); 

    // Text content 
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(4)
       << "d = (" << dx << ", " << dy << ", " << dz << ")\n"
       << "|d| = " << dist<<" mm";

    this->TranslationTextActor->SetInput(ss.str().c_str());

    double mid[3] = {
        0.5 * (p1[0] + p2[0]),
        0.5 * (p1[1] + p2[1]),
        0.5 * (p1[2] + p2[2])
    };
    this->TranslationTextActor->SetPosition(mid);

    auto clamp01 = [](double v) -> double
    {
        if (v < 0.0) { return 0.0; }
        if (v > 1.0) { return 1.0; }
        return v;
    };

    double t = 0.0;
    if (this->TranslationBillboardDistForMax > 1e-12)
    {
        t = clamp01(dist / this->TranslationBillboardDistForMax);
    }

    const double mul =
        this->TranslationBillboardMinMul +
        (this->TranslationBillboardMaxMul - this->TranslationBillboardMinMul) * t;

    const double targetWorldHeight =
        this->ComputeScalingFactor(this->TranslationBillboardPixelHeightPx) * mul;

    double bounds[6];
    this->TranslationTextActor->GetBounds(bounds);

    const double h = bounds[5] - bounds[4];
    if (h > 1e-12 && std::isfinite(h))
    {
        const double s = targetWorldHeight / h;
        this->TranslationTextActor->SetScale(s, s, s);
    }

    if (ren && ren->GetRenderWindow())
    {
        ren->GetRenderWindow()->Render();
    }
}

void vtkTransformationWidget::AttachRotationOverlayToRenderer(vtkRenderer* ren)
{
    if (!ren)
    {
        return;
    }

    this->CreateRotationOverlayIfNeeded();

    if (this->RotationTextActor && !ren->HasViewProp(this->RotationTextActor))
    {
        ren->AddViewProp(this->RotationTextActor);
    }
}

void vtkTransformationWidget::DetachRotationOverlayFromRenderer(vtkRenderer* ren)
{
    if (!ren)
    {
        return;
    }

    if (this->RotationTextActor && ren->HasViewProp(this->RotationTextActor))
    {
        ren->RemoveViewProp(this->RotationTextActor);
    }
}

void vtkTransformationWidget::CreateRotationOverlayIfNeeded()
{
    vtkRenderer* ren = this->GetCurrentRenderer();

    if (this->RotationTextActor)
    {
        if (ren && !ren->HasViewProp(this->RotationTextActor))
        {
            ren->AddViewProp(this->RotationTextActor);
        }
        return;
    }

    double o[3];
    this->widgetCS.GetOrigin(o);

    this->RotationTextActor = vtkSmartPointer<vtkBillboardTextActor3D>::New();
    this->RotationTextActor->PickableOff();
    this->RotationTextActor->SetInput("");
    this->RotationTextActor->SetPosition(o[0], o[1], o[2]);
    this->RotationTextActor->SetVisibility(false);

    if (vtkTextProperty* tp = this->RotationTextActor->GetTextProperty())
    {
        tp->SetColor(1.0, 1.0, 1.0);
        tp->SetBackgroundColor(0.0, 0.0, 0.0);
        tp->SetBackgroundOpacity(0.25);
        tp->SetBold(1);
    }
}

void vtkTransformationWidget::BeginRotationOverlay(const double centerWorld[3])
{
    this->CreateRotationOverlayIfNeeded();
	this->SetOcclusionSphereVisible(false);


    this->RotationOverlayActive = true;

    if (this->RotationTextActor)
    {
        this->RotationTextActor->SetVisibility(true);
        this->RotationTextActor->SetPosition(centerWorld[0], centerWorld[1], centerWorld[2]);
        this->RotationTextActor->SetInput("theta = 0.00 deg");
    }

    this->RefreshInteractor();
}

void vtkTransformationWidget::EndRotationOverlay()
{
    this->RotationOverlayActive = false;
	this->SetOcclusionSphereVisible(true);
	
    if (this->RotationTextActor)
    {
        this->RotationTextActor->SetVisibility(false);
        this->RotationTextActor->SetInput("");
    }

    this->RefreshInteractor();
}

void vtkTransformationWidget::UpdateRotationOverlay(double thetaDeg, const double centerWorld[3], const double hitWorld[3])
{
    if (!this->RotationOverlayActive)
    {
        return;
    }
    if (!this->RotationTextActor)
    {
        return;
    }

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(2) << "theta = " << thetaDeg << " deg";
    this->RotationTextActor->SetInput(ss.str().c_str());

    // Place billboard slightly OUTSIDE the ring along radial direction (center -> hit)
    double radial[3] = {
        hitWorld[0] - centerWorld[0],
        hitWorld[1] - centerWorld[1],
        hitWorld[2] - centerWorld[2]
    };

    double rlen2 = vtkMath::Dot(radial, radial);
    double pos[3] = { hitWorld[0], hitWorld[1], hitWorld[2] };

    if (rlen2 > 1e-12)
    {
        double rlen = std::sqrt(rlen2);
        vtkMath::MultiplyScalar(radial, 1.0 / rlen);

        const double outMul = 1.15; // push outside ring
        pos[0] = centerWorld[0] + radial[0] * (rlen * outMul);
        pos[1] = centerWorld[1] + radial[1] * (rlen * outMul);
        pos[2] = centerWorld[2] + radial[2] * (rlen * outMul);
    }

    // Nudge toward camera to reduce z-fighting
    double dop[3] = { 0.0, 0.0, 1.0 };
    vtkRenderer* ren = this->GetCurrentRenderer();
    if (ren && ren->GetActiveCamera())
    {
        ren->GetActiveCamera()->GetDirectionOfProjection(dop);
        vtkMath::Normalize(dop);
    }

    double eps = 1e-3;
    if (ren && ren->GetActiveCamera())
    {
        double camPos[3];
        ren->GetActiveCamera()->GetPosition(camPos);
        const double camDist = std::sqrt(vtkMath::Distance2BetweenPoints(camPos, centerWorld));
        eps = std::max(eps, camDist * 1e-6);
    }

    pos[0] -= dop[0] * eps;
    pos[1] -= dop[1] * eps;
    pos[2] -= dop[2] * eps;

    this->RotationTextActor->SetPosition(pos[0], pos[1], pos[2]);

    // Dynamic pixel-constant sizing (+ slight growth with angle magnitude)
    auto clamp01 = [](double v) -> double
    {
        if (v < 0.0) { return 0.0; }
        if (v > 1.0) { return 1.0; }
        return v;
    };

    double t = 0.0;
    if (this->RotationBillboardAngleForMaxDeg > 1e-12)
    {
        t = clamp01(std::abs(thetaDeg) / this->RotationBillboardAngleForMaxDeg);
    }

    const double mul =
        this->RotationBillboardMinMul +
        (this->RotationBillboardMaxMul - this->RotationBillboardMinMul) * t;

    const double targetWorldHeight =
        this->ComputeScalingFactor(this->RotationBillboardPixelHeightPx) * mul;

    double bounds[6];
    this->RotationTextActor->GetBounds(bounds);
    const double h = bounds[5] - bounds[4];

    if (h > 1e-12 && std::isfinite(h))
    {
        const double s = targetWorldHeight / h;
        this->RotationTextActor->SetScale(s, s, s);
    }

    if (ren && ren->GetRenderWindow())
    {
        ren->GetRenderWindow()->Render();
    }
}

void vtkTransformationWidget::ApplyInteractionColorsForPickedHandle()
{
    if (arrowXPicked) { SetHandleColor(this->ArrowXActor, X_INTERACT_COLOR); }
    if (arrowYPicked) { SetHandleColor(this->ArrowYActor, Y_INTERACT_COLOR); }
    if (arrowZPicked) { SetHandleColor(this->ArrowZActor, Z_INTERACT_COLOR); }

    if (quadXYPicked) { SetHandleColor(this->QuadXYActor, Z_INTERACT_COLOR); } 
    if (quadXZPicked) { SetHandleColor(this->QuadXZActor, Y_INTERACT_COLOR); }
    if (quadZYPicked) { SetHandleColor(this->QuadZYActor, X_INTERACT_COLOR); }

    if (rotXPicked) { SetHandleColor(this->RotXActor, X_INTERACT_COLOR); }
    if (rotYPicked) { SetHandleColor(this->RotXActor, Y_INTERACT_COLOR); }
    if (rotZPicked) { SetHandleColor(this->RotZActor, Z_INTERACT_COLOR); }

    if (centerPicked) { SetHandleColor(this->CenterSphereActor, CENTER_INTERACT_COLOR); }
}

void vtkTransformationWidget::CreateOcclusionSphere(double radius)
{
    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0.0, 0.0, 0.0);
    sphere->SetRadius(radius - 0.001);
    sphere->SetPhiResolution(48);
    sphere->SetThetaResolution(48);

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    // 1) Depth pre-pass actor 
    this->OcclusionSphereDepthActor = vtkSmartPointer<vtkDepthOnlyActor>::New();
    this->OcclusionSphereDepthActor->SetMapper(mapper);
    this->OcclusionSphereDepthActor->PickableOff();
   // this->OcclusionSphereDepthActor->ForceOpaqueOn();           // keep it in opaque pass
    this->OcclusionSphereDepthActor->GetProperty()->SetOpacity(1.0);

    // 2) Visual translucent actor (the look)
    this->OcclusionSphereActor = vtkSmartPointer<vtkActor>::New();
    this->OcclusionSphereActor->SetMapper(mapper);
    this->OcclusionSphereActor->PickableOff();

    auto* p = this->OcclusionSphereActor->GetProperty();
    p->SetColor(0.15, 0.15, 0.15);
    p->SetOpacity(0.12);
    p->SetAmbient(1.0);
    p->SetDiffuse(0.0);
    p->SetSpecular(0.0);
}

void vtkTransformationWidget::CaptureRotationStartDirWorld()
{
    double axisW[3] = { 0.0, 0.0, 1.0 };
    if (this->rotXPicked)
    {
        this->widgetCSStart.GetX(axisW);
    }
    else if (this->rotYPicked)
    {
        this->widgetCSStart.GetY(axisW);
    }
    else
    {
        this->widgetCSStart.GetZ(axisW);
    }

    if (vtkMath::Normalize(axisW) == 0.0)
    {
        this->RotStartDirWorld[0] = 1.0;
        this->RotStartDirWorld[1] = 0.0;
        this->RotStartDirWorld[2] = 0.0;
        return;
    }

    double centerW[3];
    this->widgetCSStart.GetOrigin(centerW);

    double v[3] = {
        this->LastPickPosition[0] - centerW[0],
        this->LastPickPosition[1] - centerW[1],
        this->LastPickPosition[2] - centerW[2]
    };

    const double d = vtkMath::Dot(v, axisW);
    v[0] -= d * axisW[0];
    v[1] -= d * axisW[1];
    v[2] -= d * axisW[2];

    if (vtkMath::Normalize(v) == 0.0)
    {
        // Fallback: pick any stable perpendicular direction to axisW
        double tmp[3] = { 1.0, 0.0, 0.0 };
        if (std::abs(axisW[0]) > 0.9)
        {
            tmp[0] = 0.0; tmp[1] = 1.0; tmp[2] = 0.0;
        }

        vtkMath::Cross(axisW, tmp, v);
        if (vtkMath::Normalize(v) == 0.0)
        {
            v[0] = 0.0; v[1] = 0.0; v[2] = 1.0;
        }
    }

    this->RotStartDirWorld[0] = v[0];
    this->RotStartDirWorld[1] = v[1];
    this->RotStartDirWorld[2] = v[2];
}

void vtkTransformationWidget::SetOcclusionForInteraction(bool interacting)
{
  this->InteractionInProgress = interacting;
  this->SetOcclusionSphereVisible(!interacting);
}

void vtkTransformationWidget::StartInteraction()
{
  this->SetOcclusionForInteraction(true);
  this->Superclass::StartInteraction();
}

void vtkTransformationWidget::EndInteraction()
{
  this->Superclass::EndInteraction();
  this->SetOcclusionForInteraction(false);
  this->SetOcclusionForInteraction(false);
}

void vtkTransformationWidget::GetDeltaTransform(vtkTransform* t)
{
    if (!t)
    {
        return;
    }

    double origin[3];
    this->widgetCSStart.GetOrigin(origin);

    double currentOrigin[3];
    this->widgetCS.GetOrigin(currentOrigin);

    const double dx = currentOrigin[0] - origin[0];
    const double dy = currentOrigin[1] - origin[1];
    const double dz = currentOrigin[2] - origin[2];

    t->Identity();
    t->PostMultiply();

    if (this->isAxisDragging || this->isPlaneDragging || this->isCenterDragging)
    {
        // Translation only.
        t->Translate(dx, dy, dz);
        return;
    }

    if (this->isRotDragging)
    {
        double axis[3] = { 0.0, 0.0, 1.0 };

        if (this->rotXPicked)
        {
            this->widgetCSStart.GetX(axis);
        }
        else if (this->rotYPicked)
        {
            this->widgetCSStart.GetY(axis);
        }
        else if (this->rotZPicked)
        {
            this->widgetCSStart.GetZ(axis);
        }
        else
        {
            return;
        }

        if (vtkMath::Normalize(axis) == 0.0)
        {
            return;
        }

        // Rotation around widget origin:
        // move by -origin, rotate, move back by +origin.
        t->Translate(-origin[0], -origin[1], -origin[2]);
        t->RotateWXYZ(this->m_rotationTheta, axis);
        t->Translate(origin[0], origin[1], origin[2]);

        return;
    }
}

void vtkTransformationWidget::GetInteractionStartCenter(double center[3]) const
{
    center[0] = this->widgetCSStart.origin[0];
    center[1] = this->widgetCSStart.origin[1];
    center[2] = this->widgetCSStart.origin[2];
}

