# KDevelop

To install KDevelop write the folloing in your command line
```
sudo apt-get install kdevelop
```

The remainder of this tutorial is an excerpt from the website in sources.

> **Sources**
>
> http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

KDevelop has excellent C++ support, GDB integration and does semantic syntax highlighting with individual colors for different variables. Such as [QtCreator](http://wiki.ros.org/IDEs#QtCreator), KDevelop supports opening CMake projects out of the box.

KDevelop must know the ROS environment variables, therefore start KDevelop from a terminal that has sourced your catkin workspace already.
Alternatively, create the following desktop file according to the remarks in the [general section](http://wiki.ros.org/IDEs#Reusing_your_shell.27s_environment) and mark it as executable:
```
mkdir -p $HOME/.local/share/applications/
touch $HOME/.local/share/applications/KDevelop.desktop
# Now edit the file using the editor of your choice and insert the text from the box below
chmod +x $HOME/.local/share/applications/KDevelop.desktop
```

```
[Desktop Entry]
Type=Application
Terminal=false
Exec=bash -i -c "kdevelop"
Name=KDevelop
Icon=kdevelop
```

## Building catkin packages

In order to build your packages in your catkin workspace, you can choose between two different approaches. The particular choice depends on your utilized build system
and your personal preference, whether to build the complete catkin workspace at once or
to build each package separately with KDevelop.

### Import catkin top-level workspace

The following steps describe how to import the complete catkin workspace into KDevelop.
This approach relies on the default catkin build system in ROS.

 * It is recommended to clear the catkin build folder before importing the project. You can safely remove the build folder from your catkin workspace since it will be created again during the cmake configure and build process.

 * Start KDevelop from shell or using the modified desktop shortcut mentioned above.
 * Goto the "Project" tab and select "Open / Import Project...". Switch to your catkin source space (we assume `~/catkin_ws/src` here) and select "CMakeLists.txt". Afterwards, you may replace the "Name" by your workspace name (default: "src"). Make sure that the selected "Build System" is "CMake Project Manager". Proceed by clicking on "Finish".

 * A new window should appear that contains cmake build configurations.
 * Set the "Build Directory" to the catkin build space, e.g.: ```/home/user/catkin_ws/build/```

 * Select your desired "Build type".
 * Finally, add the following '''Extra Arguments''' to make sure, that the binaries and libraries are moved into the correct catkin spaces (devel and install):

 ```-DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_INSTALL_PREFIX=../install```

 * Wait until KDevelop has finished importing your workspace. Try to compile your code by clicking on the "Build button".

 * After switching to the source code of an underlying package let KDevelop some seconds to parse for auto-completion and code highlighting.
