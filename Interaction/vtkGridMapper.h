////////////////////////////////////////////////////////////////////////////////
//
// vtkGridMapper - mapper for a plane actor projecting
// an infinite grid onto wcs planes 
//
//  @author:
//    Philipp Huebner 
// 
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Util.h"
#include <array>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLCamera.h>
#include <vtkShaderProgram.h>
#include <vtkOpenGLVertexArrayObject.h>
#include <vtkOpenGLVertexBufferObjectGroup.h>
#include <vtkActor.h>
#include <vtkInformation.h>
#include <vtkOpenGLRenderPass.h>
#include <vtkSmartPointer.h>

#include "vtkUtilsModule.h" // For export macro


        class VTKUTILS_EXPORT  vtkGridMapper : public vtkOpenGLPolyDataMapper
        {
        public:
            static vtkGridMapper* New();
            vtkTypeMacro(vtkGridMapper, vtkOpenGLPolyDataMapper);

            void PrintSelf(ostream& os, vtkIndent indent) override;

            // Set spacing
            vtkSetMacro(grid1Spacing, float);
            vtkSetMacro(grid2Spacing, float);
            vtkSetMacro(grid3Spacing, float);	
			vtkSetMacro(grid1FadeStartDistance, float);
            vtkSetMacro(grid1FadeEndDistance, float);
			vtkSetMacro(grid2FadeStartDistance, float);
            vtkSetMacro(grid2FadeEndDistance, float);
			vtkSetMacro(grid3FadeStartDistance, float);
            vtkSetMacro(grid3FadeEndDistance, float);	
			vtkSetMacro(gridTransparency, float);
			vtkSetMacro(grid1LineWidth, float);
            vtkSetMacro(grid2LineWidth, float);	
			vtkSetMacro(grid3LineWidth, float);
            vtkSetMacro(axisXLineWidth, float);
            vtkSetMacro(axisYLineWidth, float);

            void SetPlaneOrigin(float x, float y, float z);

			// OrthoGridFade
			void SetOrthoGridFadeRow(int idx, const float values[4]) {
				if (idx < 0 || idx >= 6) return;
				for (int i = 0; i < 4; ++i)
					this->orthoGridFade[idx][i] = values[i];
				this->Modified();  // VTK convention
			}

			void GetOrthoGridFadeRow(int idx, float values[4]) const {
				if (idx < 0 || idx >= 6) return;
				for (int i = 0; i < 4; ++i)
					values[i] = this->orthoGridFade[idx][i];
			}

			// OrthoGridColorSwitch
			void SetOrthoGridColorSwitchRow(int idx, const float values[4]) {
				if (idx < 0 || idx >= 6) return;
				for (int i = 0; i < 4; ++i)
					this->orthoGridColorSwitch[idx][i] = values[i];
				this->Modified();
			}

			void GetOrthoGridColorSwitchRow(int idx, float values[4]) const {
				if (idx < 0 || idx >= 6) return;
				for (int i = 0; i < 4; ++i)
					values[i] = this->orthoGridColorSwitch[idx][i];
			}
			
			// Set up index
            void SetUpIndex(int index)
            {
                if (index < 1 || index > 3)
                {
                    vtkErrorMacro("Invalid index. Must be 1 (Y up), 2 (X up), or 3 (Z up).");
                    return;
                }
                this->upIndex = index;
                this->Modified();
            }
			
			void SetOrthoGridSpacing(const float spacing[3])
			{
				for (int i = 0; i < 6; ++i)
				{
					this->orthoGridSpacing[i] = spacing[i];
				}
				this->Modified();  // Standard VTK pattern
			}
			
             // Set Grid colors
            void SetGrid1Color(float r, float g, float b, float a)
            {
                this->grid1Color[0] = r;
                this->grid1Color[1] = g;
                this->grid1Color[2] = b;
                this->grid1Color[3] = a;
                this->Modified();
            };

            void SetGrid2Color(float r, float g, float b, float a)
            {
                this->grid2Color[0] = r;
                this->grid2Color[1] = g;
                this->grid2Color[2] = b;
                this->grid2Color[3] = a;
                this->Modified();
            };

            void SetGrid3Color(float r, float g, float b, float a)
            {
                this->grid3Color[0] = r;
                this->grid3Color[1] = g;
                this->grid3Color[2] = b;
                this->grid3Color[3] = a;
                this->Modified();
            };

            // Set axis colors
            void SetXAxisColor(float r, float g, float b, float a);
            void SetYAxisColor(float r, float g, float b, float a);
            void SetZAxisColor(float r, float g, float b, float a);

            // Set solid mode
            vtkSetMacro(SolidMode, int);            // 0 = regular grid, 1 = solid plane
            vtkGetMacro(SolidMode, int);
            void SetSolidColor(float r, float g, float b, float a)
            {
                this->SolidColor[0] = r; this->SolidColor[1] = g;
                this->SolidColor[2] = b; this->SolidColor[3] = a;
                this->Modified();
            }

            //  override GetBounds() to provide a large bounding box, so the grid 
            // is not culled by the frustum.
            using vtkOpenGLPolyDataMapper::GetBounds;
            double* GetBounds() override;

            virtual bool HasTranslucentPolygonalGeometry() override { return true; }

            // Disallow copy/assignment
            vtkGridMapper(const vtkGridMapper&) = delete;
            void operator=(const vtkGridMapper&) = delete;


        protected:
            vtkGridMapper();
            ~vtkGridMapper() override = default;

            // Substitutes custom shader code 
            void ReplaceShaderValues(
                std::map<vtkShader::Type, vtkShader*> shaders,
                vtkRenderer* ren,
                vtkActor* actor) override;

            // Sets uniforms
            void SetMapperShaderParameters(
                vtkOpenGLHelper& cellBO,
                vtkRenderer* ren,
                vtkActor* actor) override;

            // Sets camera-related uniforms
            void SetCameraShaderParameters(
                vtkOpenGLHelper& cellBO,
                vtkRenderer* ren,
                vtkActor* actor) override;

            // Build a fullscreen quad for the infinite grid plane
            void BuildBufferObjects(
                vtkRenderer* ren,
                vtkActor* act) override;

            // Main render entry point
            void RenderPiece(
                vtkRenderer* ren,
                vtkActor* actor) override;

            // Invalidate shaders if necessary
            bool GetNeedToRebuildShaders(
                vtkOpenGLHelper& cellBO,
                vtkRenderer* ren,
                vtkActor* act) override;

            void BuildShaders(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* act) override;

            virtual void ReplaceShaderCustomUniforms(std::map<vtkShader::Type, vtkShader*> shaders, vtkActor* act) override;
            virtual void ReplaceShaderPositionVC(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* act) override;
            virtual void ReplaceShaderDepth(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* act) override;
            virtual void ReplaceShaderColor(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* act) override;

            // Grid fading (1 is the outer grid)
            float grid1FadeStartDistance = 8.0f;
            float grid1FadeEndDistance = 25.0f;

            float grid2FadeStartDistance = 1.0f;
            float grid2FadeEndDistance = 5.0f;

            float grid3FadeStartDistance = 0.10f;
            float grid3FadeEndDistance = 0.5f;

            float grid1LineWidth = 1.0f;
            float grid2LineWidth = 1.0f;
            float grid3LineWidth = 1.0f;

            // Grid spacing (must be a multiple of inner spacing) (1 is the outer grid)
            float grid1Spacing = 10.0f;
            float grid2Spacing = 100.0f;
            float grid3Spacing = 1000.0f;

            float PlaneOrigin[3] = {0.0f, 0.0f, 0.0f};

            // Grid color (1 is the outer grid)
            float gridTransparency = 0.7f;
            //float grid1Color[4] = { 1.0f, 0.0f, 0.0f, 0.5f };
            //float grid2Color[4] = { 0.0f, 1.0f, 0.0f, 1.0f };
            //float grid3Color[4] = { 0.0f, 0.0f, 1.0f, 0.9f };
            float grid1Color[4] = { 0.5f, 0.5f, 0.5f, 0.9f };
            float grid2Color[4] = { 0.4f, 0.4f, 0.4f, 0.8f };
            float grid3Color[4] = { 0.3f, 0.3f, 0.3f, 0.5f };

            float orthoGrid1Color[4] = { 0.3f, 0.3f, 0.3f, 0.2f };
            float orthoGrid2Color[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
            float orthoGrid3Color[4] = { 0.8f, 0.8f, 0.8f, 0.9f };

            float xAxisColor[4] = { 1.0f, 0.0f, 0.0f, 1.0f }; // Default: Red
            float yAxisColor[4] = { 0.0f, 1.0f, 0.0f, 1.0f }; // Default: Green
            float zAxisColor[4] = { 0.0f, 0.0f, 1.0f, 1.0f }; // Default: Blue

            // Fade Angle for ortho 
            Util::Radian orthoFadeAngle = Util::Degree(10.0f);
            // Fade in: (0 -> 2)
            // Major: (1 -> 2 -> 3 -> 4)
            // Fade out: (3 -> 5)
            std::array<std::array<float, 4>, 6> orthoGridFade;
            std::array<float, 6> orthoGridSpacing = {.1f, 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f };
            std::array<std::array<float, 4>, 6> orthoGridColorSwitch;

            int upIndex = 1;

            int   SolidMode = 0;                    // default off
            float SolidColor[4] = {0.12f, 0.12f, 0.12f, 1.0f};

            float axisXLineWidth = 1.0f;  // thickness for inner X axis (red)
            float axisYLineWidth = 1.0f;  // thickness for inner Y axis (green)
        };



