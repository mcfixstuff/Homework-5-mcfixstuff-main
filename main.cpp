#include <iostream>
#include <string>
#include "common.h"

int main() {
#ifdef _WIN32
    std::string obj1 = "../iso_sphere";
    std::string obj2 = "../cow";
#else
    std::string obj1 = "iso_sphere";
    std::string obj2 = "cow";
#endif
    loop_subdivision::subdivideMesh(obj1 + ".obj", obj1 + "_subdivided.obj");
    loop_subdivision::subdivideMesh(obj2 + ".obj", obj2 + "_subdivided.obj");
    qem::simplifyMesh(obj1 + ".obj", obj1 + "_simplified.obj");
    qem::simplifyMesh(obj2 + ".obj", obj2 + "_simplified.obj");
    std::cout << "Done." << std::endl;
    return 0;
}