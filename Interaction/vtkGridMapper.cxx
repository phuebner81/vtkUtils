////////////////////////////////////////////////////////////////////////////////
//
// vtkGridMapper - mapper for a plane actor projecting
// an infinite grid onto wcs planes (Blender/Houdini like)
//
//  @author:
//    Philipp Huebner 
// 
////////////////////////////////////////////////////////////////////////////////

#include <vtkRenderer.h>
#include <vtkMatrix4x4.h>
#include <vtkMatrix3x3.h>
#include <vtkFloatArray.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLVertexBufferObjectGroup.h>
#include <vtkShaderProgram.h>
#include <vtkOpenGLCamera.h>
#include <vtkRenderingOpenGLConfigure.h>
#include <vtk_glew.h>
#include <vtkOpenGLHelper.h>

#include "vtkGridMapper.h"

vtkStandardNewMacro(vtkGridMapper);

//----------------------------------------------------------------------------
vtkGridMapper::vtkGridMapper()
{
  // Build default ortho fades (6 tiers)
  float fadeFactor = 22.0f;
  float rangeFactor = 1.5f;
  float unit = 16.0f;
  int i = 0;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;
  orthoGridFade[i++] = { unit * (fadeFactor * rangeFactor), unit * rangeFactor, unit / rangeFactor, unit / (fadeFactor * rangeFactor) }; unit /= 16.0f;

  this->StaticOn();
}

//----------------------------------------------------------------------------
void vtkGridMapper::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void vtkGridMapper::ReplaceShaderCustomUniforms(std::map<vtkShader::Type, vtkShader*> shaders, vtkActor* actor)
{
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

  // defines uniforms, helpers, grid/axis logic
  std::string replacement;
  replacement +=
      "#define M_PI 3.1415926535897932384626433832795\n"
      "#define M_HALFPI 1.5707963267948966192313216916398\n"
      "\n"
      "uniform float grid1Spacing;\n"
      "uniform float grid2Spacing;\n"
      "uniform float grid3Spacing;\n"
      "uniform float grid1FadeStartDistance;\n"
      "uniform float grid1FadeEndDistance;\n"
      "uniform float grid2FadeStartDistance;\n"
      "uniform float grid2FadeEndDistance;\n"
      "uniform float grid3FadeStartDistance;\n"
      "uniform float grid3FadeEndDistance;\n"
      "uniform float gridTransparency;\n"
      "uniform vec4 grid1Color;\n"
      "uniform vec4 grid2Color;\n"
      "uniform vec4 grid3Color;\n"
      "uniform vec4 xAxisColor;\n"
      "uniform vec4 yAxisColor;\n"
      "uniform vec4 zAxisColor;\n"
      "uniform float grid1LineWidth;\n"
      "uniform float grid2LineWidth;\n"
      "uniform float grid3LineWidth;\n"
      "uniform int  solidMode;\n"
      "uniform vec3 planeOrigin;\n"
      "uniform float zoomLevel;\n"
      "uniform float orthoFadeAngle;\n"
      "uniform mat4 WCDCMatrix;\n"
      "uniform float mode;\n"
      "uniform vec3 cameraPos;\n"
      "uniform vec3 cameraDir;\n"
      "uniform int upIndex;\n"
      // NEW: per-axis inner line widths
      "uniform float axisXLineWidth;\n"
      "uniform float axisYLineWidth;\n"
      "\n"
      "in vec3 nearPoint;\n"
      "in vec3 farPoint;\n"
      "\n";

  for (size_t i = 0; i < orthoGridFade.size(); i++)
    replacement += "uniform vec4 orthoGrid" + Util::StringConverter::toString(i) + "Fade;\n";
  for (size_t i = 0; i < orthoGridColorSwitch.size(); i++)
    replacement += "uniform vec4 orthoGrid" + Util::StringConverter::toString(i) + "ColorSwitch;\n";
  for (size_t i = 0; i < orthoGridSpacing.size(); i++)
    replacement += "uniform float orthoGrid" + Util::StringConverter::toString(i) + "Spacing;\n";

  // Utility methods for grid rendering
  replacement +=
      "float pristineGrid(in vec2 uv, vec2 lineWidth)\n"
      "{\n"
      "    vec2 ddx = dFdx(uv);\n"
      "    vec2 ddy = dFdy(uv);\n"
      "    vec2 uvDeriv = vec2(length(vec2(ddx.x, ddy.x)), length(vec2(ddx.y, ddy.y)));\n"
      "    bvec2 invertLine = bvec2(lineWidth.x > 0.5, lineWidth.y > 0.5);\n"
      "    vec2 targetWidth = vec2(\n"
      "      invertLine.x ? 1.0 - lineWidth.x : lineWidth.x,\n"
      "      invertLine.y ? 1.0 - lineWidth.y : lineWidth.y\n"
      "    );\n"
      "    vec2 drawWidth = clamp(targetWidth, uvDeriv, vec2(0.5));\n"
      "    vec2 lineAA = uvDeriv * 1.5;\n"
      "    vec2 gridUV = abs(fract(uv) * 2.0 - 1.0);\n"
      "    gridUV.x = invertLine.x ? gridUV.x : 1.0 - gridUV.x;\n"
      "    gridUV.y = invertLine.y ? gridUV.y : 1.0 - gridUV.y;\n"
      "    vec2 grid2 = smoothstep(drawWidth + lineAA, drawWidth - lineAA, gridUV);\n"
      "    \n"
      "    grid2 *= clamp(targetWidth / drawWidth, 0.0, 1.0);\n"
      "    grid2 = mix(grid2, targetWidth, clamp(uvDeriv * 2.0 - 1.0, 0.0, 1.0));\n"
      "    grid2.x = invertLine.x ? 1.0 - grid2.x : grid2.x;\n"
      "    grid2.y = invertLine.y ? 1.0 - grid2.y : grid2.y;\n"
      "    return max(grid2.x, grid2.y);\n"
      "}\n";

replacement +=
    "float grid(vec3 fragPos3D, float scale, float lineThicknessFactor)\n"
    "{\n"
    "    vec2 coord;\n"
    "    if (upIndex == 1)      { coord = (fragPos3D.xz - planeOrigin.xz) * scale; }\n"
    "    else if (upIndex == 2) { coord = (fragPos3D.yz - planeOrigin.yz) * scale; }\n"
    "    else                   { coord = (fragPos3D.xy - planeOrigin.xy) * scale; }\n"
    "    return pristineGrid(coord, vec2(lineThicknessFactor * scale));\n"
    "}\n\n";


  replacement +=
      "vec2 pristineGridAxis(in vec2 uv, vec2 lineWidth)\n"
      "{\n"
      "    vec2 ddx = dFdx(uv);\n"
      "    vec2 ddy = dFdy(uv);\n"
      "    vec2 uvDeriv = vec2(length(vec2(ddx.x, ddy.x)), length(vec2(ddx.y, ddy.y)));\n"
      "    bvec2 invertLine = bvec2(lineWidth.x > 0.5, lineWidth.y > 0.5);\n"
      "    vec2 targetWidth = vec2(\n"
      "      invertLine.x ? 1.0 - lineWidth.x : lineWidth.x,\n"
      "      invertLine.y ? 1.0 - lineWidth.y : lineWidth.y\n"
      "    );\n"
      "    vec2 drawWidth = clamp(targetWidth, uvDeriv, vec2(0.5));\n"
      "    vec2 lineAA = uvDeriv * 1.5;\n"
      "    vec2 axisLines2 = smoothstep(drawWidth + lineAA, drawWidth - lineAA, abs(uv * 2.0));\n"
      "    axisLines2 *= clamp(lineWidth / drawWidth, 0.0, 1.0);\n"
      "    axisLines2 = mix(axisLines2, targetWidth, clamp(uvDeriv * 2.0 - 1.0, 0.0, 1.0));\n"
      "    axisLines2.x = invertLine.x ? 1.0 - axisLines2.x : axisLines2.x;\n"
      "    axisLines2.y = invertLine.y ? 1.0 - axisLines2.y : axisLines2.y;\n"
      "    return axisLines2;\n"
      "}\n";


replacement +=
    "vec2 gridAxis(vec3 fragPos3D, float scale, float lineThicknessFactorX, float lineThicknessFactorY)\n"
    "{\n"
    "    vec2 coord;\n"
    "    if (upIndex == 1)      { coord = (fragPos3D.xz - planeOrigin.xz) * scale; }\n"
    "    else if (upIndex == 2) { coord = (fragPos3D.yz - planeOrigin.yz) * scale; }\n"
    "    else                   { coord = (fragPos3D.xy - planeOrigin.xy) * scale; }\n"
    "    return pristineGridAxis(coord, vec2(lineThicknessFactorX * scale, lineThicknessFactorY * scale));\n"
    "}\n\n";


  replacement +=
      "// up-vector dependent frag pos (ray-plane intersection with translation)\n"
      "vec3 getFragPos3D(vec3 nearPoint, vec3 farPoint, int upIndex)\n"
      "{\n"
      "    vec3 diff = farPoint - nearPoint;\n"
      "    float t = 0.0;\n"
      "    float eps = 1e-6;\n"
      "    if (upIndex == 1) {\n"
      "        float denom = diff.y; if (abs(denom) < eps) discard;\n"
      "        t = (planeOrigin.y - nearPoint.y) / denom;\n"
      "    } else if (upIndex == 2) {\n"
      "        float denom = diff.x; if (abs(denom) < eps) discard;\n"
      "        t = (planeOrigin.x - nearPoint.x) / denom;\n"
      "    } else {\n"
      "        float denom = diff.z; if (abs(denom) < eps) discard;\n"
      "        t = (planeOrigin.z - nearPoint.z) / denom;\n"
      "    }\n"
      "    if (t < 0.0) { discard; }\n"
      "    vec3 fragPos3D = nearPoint + t * diff;\n"
      "    if (upIndex == 1) fragPos3D.y = planeOrigin.y;\n"
      "    else if (upIndex == 2) fragPos3D.x = planeOrigin.x;\n"
      "    else fragPos3D.z = planeOrigin.z;\n"
      "    return fragPos3D;\n"
      "}\n\n";

replacement +=
    "vec3 getFragPos3DOrtho(vec3 nearPoint, vec3 farPoint)\n"
    "{\n"
    "    vec3 diff = farPoint - nearPoint;\n"
    "    float t = 0.0;\n"
    "    float eps = 1e-6;\n"
    "    if (upIndex == 1) {\n"
    "        float denom = diff.y; if (abs(denom) < eps) discard;\n"
    "        t = (planeOrigin.y - nearPoint.y) / denom;\n"
    "    } else if (upIndex == 2) {\n"
    "        float denom = diff.x; if (abs(denom) < eps) discard;\n"
    "        t = (planeOrigin.x - nearPoint.x) / denom;\n"
    "    } else {\n"
    "        float denom = diff.z; if (abs(denom) < eps) discard;\n"
    "        t = (planeOrigin.z - nearPoint.z) / denom;\n"
    "    }\n"
    "    vec3 fragPos3D = nearPoint + t * diff;\n"
    "    if (upIndex == 1)      fragPos3D.y = planeOrigin.y;\n"
    "    else if (upIndex == 2) fragPos3D.x = planeOrigin.x;\n"
    "    else                   fragPos3D.z = planeOrigin.z;\n"
    "    return fragPos3D;\n"
    "}\n";


  replacement +=
      "float computeDepth(vec3 pos)\n"
      "{\n"
      "  float f = gl_DepthRange.far;\n"
      "  float n = gl_DepthRange.near;\n"
      "  vec4 clipPos = WCDCMatrix * vec4(pos, 1.0);\n"
      "  float ndcDepth = clipPos.z / clipPos.w;\n"
      "  return (((f - n) * ndcDepth) + n + f) / 2.0;\n"
      "}\n\n";

  replacement +=
      "float computeLineScaleFactor(vec3 pos, vec3 cameraPos, float startDistance, float endDistance, float minFactor)\n"
      "{\n"
      "  float d = distance(pos, cameraPos);\n"
      "  return mix(minFactor, 1.0, smoothstep(startDistance, endDistance, d));\n"
      "}\n";

  replacement +=
      "float computeFade(vec3 pos, vec3 cameraPos, float fadeStartDistance, float fadeEndDistance)\n"
      "{\n"
      "  float d = distance(pos, cameraPos);\n"
      "  return 1.0 - smoothstep(fadeStartDistance, fadeEndDistance, d);\n"
      "}\n";

  replacement +=
      "vec4 computeOrthoFade(float d, float fadeInStart, float fadeInEnd, float fadeOutStart, float fadeOutEnd, vec4 colorSwitch)\n"
      "{\n"
      "  float fadeIn = 1.0 - smoothstep(fadeInEnd, fadeInStart, d);\n"
      "  float fadeOut = smoothstep(fadeOutEnd, fadeOutStart, d);\n"
      "  float main = pow(fadeIn * fadeOut, 0.7);\n"
      "  float colorIn  = smoothstep(colorSwitch.y, colorSwitch.x, d);\n"
      "  float colorOut = 1.0 - smoothstep(colorSwitch.w, colorSwitch.z, d);\n"
      "  return vec4(colorIn, (1.0 - colorIn) * (1.0 - colorOut), colorOut, main);\n"
      "}\n";

  replacement +=
      "vec4 getAxisColor(vec2 axisFactor)\n"
      "{\n"
      "    vec4 axisColor1 = vec4(0.0);\n"
      "    vec4 axisColor2 = vec4(0.0);\n"
      "    if (upIndex == 1)      axisColor1 = zAxisColor;\n"
      "    else if (upIndex == 2) axisColor1 = zAxisColor;\n"
      "    else                    axisColor1 = yAxisColor;\n"
      "    if (upIndex == 1)      axisColor2 = xAxisColor;\n"
      "    else if (upIndex == 2) axisColor2 = yAxisColor;\n"
      "    else                    axisColor2 = xAxisColor;\n"
      "    axisColor1 = axisColor1 * axisFactor.x;\n"
      "    axisColor2 = axisColor2;\n"
      "    return mix(axisColor1, axisColor2, axisFactor.y);\n"
      "}\n";

  vtkShaderProgram::Substitute(FSSource, "//VTK::CustomUniforms::Dec", replacement);
  shaders[vtkShader::Fragment]->SetSource(FSSource);

  this->Superclass::ReplaceShaderCustomUniforms(shaders, actor);
}

//------------------------------------------------------------------------------
void vtkGridMapper::ReplaceShaderPositionVC(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  std::string VSSource = shaders[vtkShader::Vertex]->GetSource();
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

  vtkShaderProgram::Substitute(VSSource, "//VTK::PositionVC::Dec",
    "uniform mat4 invWCVCMatrix;\n"
    "uniform mat4 invVCDCMatrix;\n"
    "\n"
    "vec3 getGridPlaneVertex(int vertexID)\n"
    "{\n"
    "    vec3 gridPlane[6] = vec3[](\n"
    "        vec3(1.1, 1.1, 0.0),\n"
    "        vec3(-1.1, -1.1, 0.0),\n"
    "        vec3(-1.1, 1.1, 0.0),\n"
    "        vec3(-1.1, -1.1, 0.0),\n"
    "        vec3(1.1, 1.1, 0.0),\n"
    "        vec3(1.1, -1.1, 0.0)\n"
    "    );\n"
    "    return gridPlane[vertexID];\n"
    "}\n"
    "\n"
    "out vec3 nearPoint;\n"
    "out vec3 farPoint;\n"
    "\n"
    "vec3 UnprojectPoint(float x, float y, float z)\n"
    "{\n"
    "    vec4 clipSpace = vec4(x, y, z, 1.0);\n"
    "    vec4 viewSpace = invVCDCMatrix * clipSpace;\n"
    "    viewSpace /= viewSpace.w;\n"
    "    vec4 worldSpace = invWCVCMatrix * viewSpace;\n"
    "    return worldSpace.xyz;\n"
    "}\n"
  );

  vtkShaderProgram::Substitute(VSSource, "//VTK::PositionVC::Impl",
    "  vec3 p = getGridPlaneVertex(gl_VertexID);\n"
    "  nearPoint = UnprojectPoint(p.x, p.y, -1.0);\n"
    "  farPoint  = UnprojectPoint(p.x, p.y,  1.0);\n"
    "  gl_Position = vec4(p, 1.0);\n"
  );

  vtkShaderProgram::Substitute(FSSource, "//VTK::PositionVC::Impl",
    "  bool isPerspective = (mode < 0.5);\n"
    "  vec3 fragPos3D;\n"
    "  if(isPerspective)\n"
    "  {\n"
    "    fragPos3D = getFragPos3D(cameraPos, farPoint, upIndex);\n"
    "  }\n"
    "  else\n"
    "  {\n"
    "    fragPos3D = getFragPos3DOrtho(nearPoint, farPoint);\n"
    "  }\n"
  );

  shaders[vtkShader::Vertex]->SetSource(VSSource);
  shaders[vtkShader::Fragment]->SetSource(FSSource);

  this->Superclass::ReplaceShaderPositionVC(shaders, ren, actor);
}

//------------------------------------------------------------------------------
void vtkGridMapper::ReplaceShaderDepth(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();
  vtkShaderProgram::Substitute(FSSource, "//VTK::Depth::Impl",
    "  gl_FragDepth = computeDepth(fragPos3D);\n"
  );
  shaders[vtkShader::Fragment]->SetSource(FSSource);
  this->Superclass::ReplaceShaderDepth(shaders, ren, actor);
}

//------------------------------------------------------------------------------
void vtkGridMapper::ReplaceShaderColor(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

  // Perspective branch
  std::string replacementPers;
  replacementPers +=
    "    //////////////////////////////\n"
    "    // PERSPECTIVE MODE\n"
    "    //////////////////////////////\n"
    "    float lineScaleFactor = 1.0 / 2.0;\n"
    "    float lineFactor = computeLineScaleFactor(fragPos3D, cameraPos, 0.0, 2.0, 0.25);\n"
    "    vec2 axisFactor = gridAxis(\n"
    "      fragPos3D,\n"
    "      grid1Spacing,\n"
    "      axisXLineWidth * lineScaleFactor * lineFactor * 3.0,\n"
    "      axisYLineWidth * lineScaleFactor * lineFactor * 3.0\n"
    "    );\n"
    "    float maxAxisFactor = max(axisFactor.x, axisFactor.y);\n"
    "    {\n"
    "      float fadeFactor = computeFade(fragPos3D, cameraPos, grid3FadeStartDistance, grid3FadeEndDistance);\n"
    "      float gridFactor = grid(fragPos3D, grid3Spacing, grid3LineWidth * lineScaleFactor * lineFactor);\n"
    "      gridFactor = max(gridFactor - maxAxisFactor, 0.0);\n"
    "      vec4 col = grid3Color;\n"
    "      finalColor = mix(finalColor, col, fadeFactor * gridFactor);\n"
    "    }\n"
    "    {\n"
    "      float fadeFactor = computeFade(fragPos3D, cameraPos, grid2FadeStartDistance, grid2FadeEndDistance);\n"
    "      float gridFactor = grid(fragPos3D, grid2Spacing, grid2LineWidth * lineScaleFactor * lineFactor);\n"
    "      gridFactor = max(gridFactor - maxAxisFactor, 0.0);\n"
    "      vec4 col = grid2Color;\n"
    "      finalColor = mix(finalColor, col, fadeFactor * gridFactor);\n"
    "    }\n"
    "    {\n"
    "      float fadeFactor = computeFade(fragPos3D, cameraPos, grid1FadeStartDistance, grid1FadeEndDistance);\n"
    "      float gridFactor = grid(fragPos3D, grid1Spacing, grid1LineWidth * lineScaleFactor * lineFactor);\n"
    "      gridFactor = max(gridFactor - maxAxisFactor, 0.0);\n"
    "      vec4 col = grid1Color;\n"
    "      finalColor = mix(finalColor, col, fadeFactor * gridFactor);\n"
    "    }\n"
    "    vec4 axisColor = getAxisColor(axisFactor);\n"
    "    finalColor = mix(finalColor, axisColor, axisColor.a);\n"
    "    finalColor.a *= gridTransparency;\n";

  // Ortho branch
  std::string replacementOrtho;
  replacementOrtho +=
    "    //////////////////////////////\n"
    "    // ORTHO MODE\n"
    "    //////////////////////////////\n"
    "    float lineScaleFactor = 1.0 / 200.0 * zoomLevel;\n"
    "    float axisLineScaleX = max(axisXLineWidth * lineScaleFactor, 0.003);\n"
    "    float axisLineScaleY = max(axisYLineWidth * lineScaleFactor, 0.003);\n"
    "    vec2 axisFactor = gridAxis(fragPos3D, orthoGrid0Spacing, axisLineScaleX, axisLineScaleY);\n"
    "    float maxAxisFactor = max(axisFactor.x, axisFactor.y);\n";

  for (size_t i = 0; i < orthoGridSpacing.size(); i++)
  {
    std::string idx = Util::StringConverter::toStdstring(orthoGridSpacing.size() - i - 1);
    replacementOrtho +=
      "    {\n"
      "      vec4 fadeFactor = computeOrthoFade(zoomLevel, orthoGrid" + idx + "Fade.x, orthoGrid" + idx + "Fade.y, orthoGrid" + idx + "Fade.z, orthoGrid" + idx + "Fade.w, orthoGrid" + idx + "ColorSwitch);\n"
      "      float gridFactor = grid(fragPos3D, orthoGrid" + idx + "Spacing, lineScaleFactor);\n"
      "      gridFactor = max(gridFactor - maxAxisFactor, 0.0);\n"
      "      vec4 gridColor = grid1Color * fadeFactor.x + grid2Color * fadeFactor.y + grid3Color * fadeFactor.z;\n"
      "      finalColor = mix(finalColor, gridColor, fadeFactor.w * gridFactor);\n"
      "    }\n";
  }

  replacementOrtho +=
    "    vec4 axisColor = getAxisColor(axisFactor);\n"
    "    finalColor = mix(finalColor, axisColor, axisColor.a);\n"
    "    if(upIndex == 1) {\n"
    "      float camToSurfaceAngle = acos(abs(dot(cameraDir, vec3(0.0, 1.0, 0.0))));\n"
    "      float angleFading = smoothstep(0.0, orthoFadeAngle, M_HALFPI - camToSurfaceAngle);\n"
    "      finalColor.a *= angleFading;\n"
    "    }\n"
    "    finalColor.a *= gridTransparency;\n";

  std::string replacement;
  replacement +=
    "  vec4 finalColor = vec4(0.0);\n"
    "  float fade1 = computeFade(fragPos3D, cameraPos, grid1FadeStartDistance, grid1FadeEndDistance);\n"
    "  float fade2 = computeFade(fragPos3D, cameraPos, grid2FadeStartDistance, grid2FadeEndDistance);\n"
    "  float fade3 = computeFade(fragPos3D, cameraPos, grid3FadeStartDistance, grid3FadeEndDistance);\n"
    "  float distanceFade = max(max(fade1, fade2), fade3);\n"
    "  if (solidMode == 1) {\n"
    "    float fadeFactor = distanceFade;\n"
    "    finalColor = vec4(0.30, 0.30, 0.30, gridTransparency * fadeFactor);\n"
    "    gl_FragData[0] = finalColor;\n"
    "    return;\n"
    "  }\n"
    "  if(isPerspective) {\n";

  replacement += replacementPers;
  replacement +=
    "  } else {\n";
  replacement += replacementOrtho;
  replacement +=
    "  }\n"
    "  gl_FragData[0] = finalColor;\n";

  vtkShaderProgram::Substitute(FSSource, "//VTK::Color::Impl", replacement);
  shaders[vtkShader::Fragment]->SetSource(FSSource);
  this->Superclass::ReplaceShaderColor(shaders, ren, actor);
}

//----------------------------------------------------------------------------
void vtkGridMapper::ReplaceShaderValues(
  std::map<vtkShader::Type, vtkShader*> shaders,
  vtkRenderer* ren,
  vtkActor* actor)
{
  this->ReplaceShaderRenderPass(shaders, ren, actor, /*prePass=*/true);
  this->ReplaceShaderCustomUniforms(shaders, actor);
  this->ReplaceShaderPositionVC(shaders, ren, actor);
  this->ReplaceShaderDepth(shaders, ren, actor);
  this->ReplaceShaderColor(shaders, ren, actor);
  this->ReplaceShaderRenderPass(shaders, ren, actor, /*prePass=*/false);
}

//----------------------------------------------------------------------------
void vtkGridMapper::SetMapperShaderParameters(
  vtkOpenGLHelper& cellBO,
  vtkRenderer* ren,
  vtkActor* actor)
{
  vtkOpenGLCamera* cam = static_cast<vtkOpenGLCamera*>(ren->GetActiveCamera());

  if (this->VBOs->GetMTime() > cellBO.AttributeUpdateTime ||
      cellBO.ShaderSourceTime > cellBO.AttributeUpdateTime ||
      cellBO.VAO->GetMTime() > cellBO.AttributeUpdateTime)
  {
    cellBO.VAO->Bind();
    this->VBOs->AddAllAttributesToVAO(cellBO.Program, cellBO.VAO);
    cellBO.AttributeUpdateTime.Modified();
  }

  cellBO.Program->SetUniformf("grid1FadeStartDistance", this->grid1FadeStartDistance);
  cellBO.Program->SetUniformf("grid1FadeEndDistance",   this->grid1FadeEndDistance);
  cellBO.Program->SetUniformf("grid2FadeStartDistance", this->grid2FadeStartDistance);
  cellBO.Program->SetUniformf("grid2FadeEndDistance",   this->grid2FadeEndDistance);
  cellBO.Program->SetUniformf("grid3FadeStartDistance", this->grid3FadeStartDistance);
  cellBO.Program->SetUniformf("grid3FadeEndDistance",   this->grid3FadeEndDistance);

  cellBO.Program->SetUniformf("grid1LineWidth", this->grid1LineWidth);
  cellBO.Program->SetUniformf("grid2LineWidth", this->grid2LineWidth);
  cellBO.Program->SetUniformf("grid3LineWidth", this->grid3LineWidth);

  cellBO.Program->SetUniformf("grid1Spacing", this->grid1Spacing);
  cellBO.Program->SetUniformf("grid2Spacing", this->grid2Spacing);
  cellBO.Program->SetUniformf("grid3Spacing", this->grid3Spacing);

  cellBO.Program->SetUniformf("gridTransparency", gridTransparency);

  bool isParallel = (cam->GetParallelProjection() != 0);
  if (isParallel)
  {
    cellBO.Program->SetUniform4f("grid1Color", orthoGrid1Color);
    cellBO.Program->SetUniform4f("grid2Color", orthoGrid2Color);
    cellBO.Program->SetUniform4f("grid3Color", orthoGrid3Color);
  }
  else
  {
    cellBO.Program->SetUniform4f("grid1Color", grid1Color);
    cellBO.Program->SetUniform4f("grid2Color", grid2Color);
    cellBO.Program->SetUniform4f("grid3Color", grid3Color);
  }

  cellBO.Program->SetUniform3fv("planeOrigin", 1, PlaneOrigin);
  cellBO.Program->SetUniformf("orthoFadeAngle", static_cast<float>(this->orthoFadeAngle.valueRadians()));

  cellBO.Program->SetUniformi("solidMode", SolidMode);

  cellBO.Program->SetUniformi("upIndex", this->upIndex);
  cellBO.Program->SetUniform4f("xAxisColor", this->xAxisColor);
  cellBO.Program->SetUniform4f("yAxisColor", this->yAxisColor);
  cellBO.Program->SetUniform4f("zAxisColor", this->zAxisColor);

  // NEW: push axis line width uniforms
  cellBO.Program->SetUniformf("axisXLineWidth", this->axisXLineWidth);
  cellBO.Program->SetUniformf("axisYLineWidth", this->axisYLineWidth);

  for (size_t i = 0; i < orthoGridFade.size(); i++)
    cellBO.Program->SetUniform4f((std::string("orthoGrid") + Util::StringConverter::toStdstring(i) + "Fade").c_str(), this->orthoGridFade[i].data());
  for (size_t i = 0; i < orthoGridColorSwitch.size(); i++)
    cellBO.Program->SetUniform4f((std::string("orthoGrid") + Util::StringConverter::toStdstring(i) + "ColorSwitch").c_str(), this->orthoGridColorSwitch[i].data());
  for (size_t i = 0; i < orthoGridSpacing.size(); i++)
    cellBO.Program->SetUniformf((std::string("orthoGrid") + Util::StringConverter::toStdstring(i) + "Spacing").c_str(), this->orthoGridSpacing[i]);

  // Render pass hook-up
  vtkInformation* info = actor->GetPropertyKeys();
  if (info && info->Has(vtkOpenGLRenderPass::RenderPasses()))
  {
    int numRenderPasses = info->Length(vtkOpenGLRenderPass::RenderPasses());
    for (int i = 0; i < numRenderPasses; ++i)
    {
      vtkObjectBase* rpBase = info->Get(vtkOpenGLRenderPass::RenderPasses(), i);
      vtkOpenGLRenderPass* rp = static_cast<vtkOpenGLRenderPass*>(rpBase);
      if (!rp->SetShaderParameters(cellBO.Program, this, actor, cellBO.VAO))
      {
        vtkErrorMacro("RenderPass::SetShaderParameters failed for renderpass: " << rp->GetClassName());
      }
    }
  }
}

//----------------------------------------------------------------------------
void vtkGridMapper::SetCameraShaderParameters(
  vtkOpenGLHelper& cellBO,
  vtkRenderer* ren,
  vtkActor* actor)
{
  vtkShaderProgram* program = cellBO.Program;
  vtkOpenGLCamera* cam = static_cast<vtkOpenGLCamera*>(ren->GetActiveCamera());

  // Key matrices
  vtkMatrix4x4* wcdc;
  vtkMatrix4x4* wcvc;
  vtkMatrix3x3* norms;
  vtkMatrix4x4* vcdc;
  cam->GetKeyMatrices(ren, wcvc, norms, vcdc, wcdc);

  vtkNew<vtkMatrix4x4> invWCVCMatrix;
  vtkMatrix4x4::Invert(wcvc, invWCVCMatrix);

  vtkNew<vtkMatrix4x4> invVCDCMatrix;
  vtkMatrix4x4::Invert(vcdc, invVCDCMatrix);

  program->SetUniformMatrix("WCDCMatrix", wcdc);
  program->SetUniformMatrix("invVCDCMatrix", invVCDCMatrix);
  program->SetUniformMatrix("invWCVCMatrix", invWCVCMatrix);

  bool isParallel = (cam->GetParallelProjection() != 0);
  program->SetUniformf("mode", isParallel ? 1.0f : 0.0f);

  float zoomLevel = 1.0f;
  if (isParallel)
  {
    zoomLevel = static_cast<float>(cam->GetParallelScale());
  }
  program->SetUniformf("zoomLevel", zoomLevel);

  double camPosD[3];
  cam->GetPosition(camPosD);
  float cameraPos[3] = { static_cast<float>(camPosD[0]), static_cast<float>(camPosD[1]), static_cast<float>(camPosD[2]) };
  program->SetUniform3fv("cameraPos", 1, cameraPos);

  double camDirD[3];
  cam->GetDirectionOfProjection(camDirD);
  float cameraDir[3] = { static_cast<float>(camDirD[0]), static_cast<float>(camDirD[1]), static_cast<float>(camDirD[2]) };
  program->SetUniform3fv("cameraDir", 1, cameraDir);
}

//----------------------------------------------------------------------------
void vtkGridMapper::BuildBufferObjects(
  vtkRenderer* ren,
  vtkActor* /*act*/)
{
  vtkNew<vtkFloatArray> quadVertices;
  quadVertices->SetNumberOfComponents(3);
  quadVertices->SetNumberOfTuples(6);

  float v0[] = { -1.f, -1.f, 0.f };
  float v1[] = {  1.f, -1.f, 0.f };
  float v2[] = { -1.f,  1.f, 0.f };
  float v3[] = { -1.f,  1.f, 0.f };
  float v4[] = {  1.f, -1.f, 0.f };
  float v5[] = {  1.f,  1.f, 0.f };

  quadVertices->SetTuple(0, v0);
  quadVertices->SetTuple(1, v1);
  quadVertices->SetTuple(2, v2);
  quadVertices->SetTuple(3, v3);
  quadVertices->SetTuple(4, v4);
  quadVertices->SetTuple(5, v5);

  vtkOpenGLRenderWindow* renWin = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
  vtkOpenGLVertexBufferObjectCache* cache = renWin->GetVBOCache();

  this->VBOs->CacheDataArray("vertexMC", quadVertices, cache, VTK_FLOAT);
  this->VBOs->BuildAllVBOs(cache);

  this->VBOBuildTime.Modified();
}

//----------------------------------------------------------------------------
void vtkGridMapper::RenderPiece(vtkRenderer* ren, vtkActor* actor)
{
  this->UpdateBufferObjects(ren, actor);
  this->UpdateShaders(this->Primitives[PrimitivePoints], ren, actor);
  glDrawArrays(GL_TRIANGLES, 0, 6);
}

void vtkGridMapper::SetXAxisColor(float r, float g, float b, float a)
{
  this->xAxisColor[0] = r; this->xAxisColor[1] = g; this->xAxisColor[2] = b; this->xAxisColor[3] = a;
  this->Modified();
}

void vtkGridMapper::SetYAxisColor(float r, float g, float b, float a)
{
  this->yAxisColor[0] = r; this->yAxisColor[1] = g; this->yAxisColor[2] = b; this->yAxisColor[3] = a;
  this->Modified();
}

void vtkGridMapper::SetZAxisColor(float r, float g, float b, float a)
{
  this->zAxisColor[0] = r; this->zAxisColor[1] = g; this->zAxisColor[2] = b; this->zAxisColor[3] = a;
  this->Modified();
}

void vtkGridMapper::SetPlaneOrigin(float x, float y, float z)
{
  this->PlaneOrigin[0] = x;
  this->PlaneOrigin[1] = y;
  this->PlaneOrigin[2] = z;
  this->Modified();
}

//----------------------------------------------------------------------------
double* vtkGridMapper::GetBounds()
{
  this->Bounds[0] = -1e6;  this->Bounds[1] =  1e6;
  this->Bounds[2] = -1e6;  this->Bounds[3] =  1e6;
  this->Bounds[4] = -1e6;  this->Bounds[5] =  1e6;
  return this->Bounds;
}

void vtkGridMapper::BuildShaders(std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* act)
{
  this->Superclass::BuildShaders(shaders, ren, act);
  // Uncomment for debugging GLSL sources if needed
  // std::string VSSource = shaders[vtkShader::Vertex]->GetSource();
  // std::string GSSource = shaders[vtkShader::Geometry]->GetSource();
  // std::string FSSource = shaders[vtkShader::Fragment]->GetSource();
}

//----------------------------------------------------------------------------
bool vtkGridMapper::GetNeedToRebuildShaders(
  vtkOpenGLHelper& cellBO,
  vtkRenderer* ren,
  vtkActor* actor)
{
  vtkMTimeType renderPassMTime = this->GetRenderPassStageMTime(actor);
  return (cellBO.Program == nullptr) || (cellBO.ShaderSourceTime < renderPassMTime);
}



