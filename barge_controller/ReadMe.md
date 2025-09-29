# Install instructions on PX4:
To install a custom plugin such as the barge controller used for my project, you must add the plugin folder ("barge_controller" in this case) to PX4-Autopilot/src/modules/simulation/gz_plugins

Then add the folder ("barge_controller" e.g.) to the CMakeLists.txt in the above directory. Additionally, add an alias in the alias section ("BargeController" e.g.)

Example text:

...

    # Add our plugins as subdirectories
    add_subdirectory(optical_flow)
    add_subdirectory(template_plugin)
    add_subdirectory(gstreamer)
    add_subdirectory(moving_platform_controller)
    add_subdirectory(generic_motor)
    add_subdirectory(buoyancy)
    add_subdirectory(spacecraft_thruster)
    add_subdirectory(barge_controller)

    # Add an alias target for each plugin
    if (TARGET GstCameraSystem)
        add_custom_target(px4_gz_plugins ALL DEPENDS OpticalFlowSystem MovingPlatformController BargeController TemplatePlugin GenericMotorModelPlugin BuoyancySystemPlugin GstCameraSystem SpacecraftThrusterModelPlugin)
    else()
        add_custom_target(px4_gz_plugins ALL DEPENDS OpticalFlowSystem MovingPlatformController BargeController TemplatePlugin GenericMotorModelPlugin BuoyancySystemPlugin SpacecraftThrusterModelPlugin)
    endif()
    
...
