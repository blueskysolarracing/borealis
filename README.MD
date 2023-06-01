# Borealis

For projects that don’t have external dependencies, open STM32CubeIDE, click build. The project should compile.

For projects that depend on `Master_Workspace/Shared_Resources/` , such as all motherboards, UARTHUB, and the BMS, you will have to do some additional work to get these projects to build.

The easy way is to run the script `fixPath.py` in the folder containing the project. For example, if you want to set up the dependencies for `Master_Workspace\Motherboards\GEN11_BBMB` , then you should run `python Master_Workspace\Motherboards\GEN11_BBMB\fixPath.py`. This should set up the build path properly in the `.project` file for your project. Then, refresh the project in your IDE, and you should be able to build by clicking build in the IDE.
