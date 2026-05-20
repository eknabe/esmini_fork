# Instructions from developer

## Initial version

I'm planning to refactor and extend functionality related to road objects.

A road object, in the context of esmini, is defined in the OpenDRIVE standard: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/index.html, chapter 13. Some introduction is found here: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_01_introduction.html

Objects can be either just a bounding box, or a group of outlines for more advanced shapes. Both bounding boxes and outlines can have a height.

An outline is defined by a set of vertices, which can be defined either as cornerRoad or cornerLocal. The vertices of an outline are of the same type. However, within the group of outlines, there might be a mix of outlines defined in cornerRoad and cornerLocal. Details on those coordinate systems are found in https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_03_object_outline.html.

Further, an object can be repeated along the road s-axis (longitudinal). While being repeated it can be linearly transformed, e.g. scaled, rotated, increase/decrease offset along road t-axis. The distance between each repeated instance is defined by a attribute. If distance is zero, it means that the object is continuous, i.e. no space between the instances, instead one solid shape. Imagine roughly a sequence of connected bounding boxes, when the road is curved the corners of the bounding boxes will overlap or be separated by gap. Extend or trim the edges so that each bounding box is a 4-corner polygon, not necessarily a perfect rectangle. There is no gap or overlap between neighbor polygons.

Lastly objects can have markings along the edges, see https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_08_object_marking.html.

The objects will be 1. visualized in the viewer and 2. reported via OSI.

Now, esmini already has implementation of some basic features:
- Parsing (from the OpenDRIVE XML)
- Creating internal representation of objects, including partial support for outline and repeat instances
- Objects are partially populated on OSI
- Object markings currently not implemented at all
- Groups of mixed and repeated outlines is not currently supported
- Tunnels are implemented as objects. This implementation I'm fairly happy about.

I think a total refactor of the implementation will make sense in order to support OpenDRIVE objects fully. Instead of patching current implementation.

I need help in this endeavor. Prioiritzed features are: 1. Repeat of continuous objects, of all types (mixed outlines, zero and no zero distance) and 2. Markings, e.g. for parking spaces.

Most of the relevant code are found here:
  parsing:
    RoadManager.hpp, lines 2452-2687
    RoadManager.cpp, lines 4943-5270
  visualization:
    roadgeom.cpp, lines 1174-1944
  osi:
    OSIReporter.cpp, lines 636-770

What do you think? Would you recommend extend current implementation or help me starting over from scratch?

## Updated version to use in case of session restart

I'm planning to refactor and extend functionality related to road objects.

A road object, in the context of esmini, is defined in the OpenDRIVE standard, which I want you to carefully consider: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/index.html, chapter 13. Some introduction is found here: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_01_introduction.html
Relevant details are found in the following sections:
https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_02_repeating_objects.html
https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_03_object_outline.html
https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_05_object_material.html
https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_07_access_rules_parking.html
https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_11_tunnels.html

Objects can be either just a bounding box, or a group of outlines for more advanced shapes. Both bounding boxes and outlines can have a height. However, if the name attribute of the object, with or without added ".osgb" extension, matches any existing file on disk, using LocateFile() function, then the 3D model should be loaded, using, LoadRoadFeature(). If 3D model loaded, also register the path with bject->SetModel3DFullPath().

In case a 3D model was loaded, the dimensions of the 3D model will be used as default. But any specified dimension (length, width, height) of the object shall override the dimension of the 3D model itself. For example, if the 3D model size (bounding box, find out using osg::ComputeBoundsVisitor) is 5 meter wide and no width is specified for the object, then width will be scaled by 1 to 5 meter. However, if the width attribute is specified, it should be applied, scaling the 3D model width (y) by factor object_width / 3D_model_width.

To clarify, any found 3D model overrules any specified outline, as well as bounding box. Whatever source for the visual representation (bounding box, outlines, 3D model) repeated instances shall be created if specified.

Note that dimension as specified in the parent/object element will override dimension in 3D file (per component length/x, width/y, height/z). But outline shape will override dimensions specified in the parent/object element.

An outline is defined by a set of vertices, which can be defined either as cornerRoad or cornerLocal. The vertices within a specific outline are of the same type. However, within the group of outlines, there might be a mix of outlines defined in cornerRoad and cornerLocal. Details on those coordinate systems are found in https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_03_object_outline.html.

Note the difference nature of Road vs Local coordinates. Road coordinates expressed in road s and t coordinates will be affected by road curvature while Local coordinates will be a fixed shape in terms of local x, y (u, v in OpenDRIVE spec) coordinates, but the origin in terms of road s, t.

Further, an object can be repeated along the road s-axis (longitudinal). While being repeated it can be linearly transformed, e.g. scaled, rotated, increase/decrease offset along road t-axis. The distance between each repeated instance is defined by a attribute. If distance is zero, it means that the object is continuous, i.e. no space between the instances, instead one solid shape. Imagine roughly a sequence of connected bounding boxes, when the road is curved the corners of the bounding boxes will overlap or be separated by gap. Extend or trim the edges so that each bounding box is a 4-corner polygon, not necessarily a perfect rectangle. There is no gap or overlap between neighbor polygons.

Note that for repeated objects, the instances will be placed along road s-axis, and hence the shape can vary between instances depending on the change of road curvature.

Regarding the z values in repeated instances: They are interpolated between segments instead of discreet plateaus.
Consider resources/tmp/road_object_vary_height.xodr
In the old, wanted behavior, the generated object height will slope along s axis. It should simply look like a slope instead of stair.

Regarding width and length. Those attributes may be overrided by optional radius attribute. Specified radius indicates a cylinder shape instead of bounding box. In case no 3D model is specified (which always wins) and no outline (which also wins over primitve shapes), then a cylindric 3D shape shall be created in viewer, preferably using OSG osg::Cylinder.

Material shall be parsed and registered, e.g. roadMarkColor will be used for object markings.

Lastly objects can have markings along the edges, see https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/1.8.0/specification/13_objects/13_08_object_marking.html. For outlines, the markings are defined per edge while for bounding box objects they are defined per side (front, left, rear, right). We can skip markings for cylinder objects.

Texture coordinates should be created for all objects, similar to how it's currently done with tunnels.

Normals needs to be calculated, preferably using osgUtil::SmoothingVisitor::smooth(geom, 0.0) for all markings. For objects we can apply osgUtil::SmoothingVisitor::smooth(geom, 0.5) to achieve some smoothing of almost coplanar neighbor faces.

Consider combining existing methods for creating roadmarks for also creating object markings. They have much in common.

Objects and any markings will be 1. visualized in the viewer and 2. reported via OSI.

Now, esmini already has implementation of some basic features:
- Parsing (from the OpenDRIVE XML)
- Creating internal representation of objects, including partial support for outline and repeat instances
- Objects are partially populated on OSI
- Tunnels are currently implemented internally as continuous repeat objects. Refactor as needed.
- Object markings currently not implemented at all, needs to be included in new implementation.
- Groups of mixed and repeated outlines is not currently supported, but needs to be included in the new implementation

I think a total refactor of the implementation will make sense in order to support OpenDRIVE objects fully. Instead of patching current implementation.

I need help in this endeavor. Prioiritzed features are: 1. Repeat of continuous objects, of all types (mixed outlines, zero and no zero distance) and 2. Markings, e.g. for parking spaces.

Most of the relevant code are found here:
  parsing:
    RoadManager.hpp, lines 2452-2687
    RoadManager.cpp, lines 4943-5270
  visualization:
    roadgeom.cpp, lines 1174-1944
  osi:
    OSIReporter.cpp, lines 636-770

What do you think? Would you recommend extend current implementation or help me starting over from scratch?
