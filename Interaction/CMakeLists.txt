#
# Source files
#
# Here is where you can add the name of your local common classes.
#
set(classes
  vtkGridMapper
  vtkTransformationWidget
)



#
# Create the vtkmyCommon C++ library.
#
vtk_module_add_module(VTKUTILS::Widgets
  CLASSES ${classes})
