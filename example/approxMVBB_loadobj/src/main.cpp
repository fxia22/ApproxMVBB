// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>
#include <fstream>

#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

static std::string GetBaseDir(const std::string& filepath) {
  if (filepath.find_last_of("/\\") != std::string::npos)
    return filepath.substr(0, filepath.find_last_of("/\\"));
  return "";
}

static bool FileExists(const std::string& abs_filename) {
  bool ret;
  FILE* fp = fopen(abs_filename.c_str(), "rb");
  if (fp) {
    ret = true;
    fclose(fp);
  } else {
    ret = false;
  }
  return ret;
}

int load_obj(std::string fn,
    tinyobj::attrib_t& attrib,
    std::vector<tinyobj::shape_t>& shapes,
    std::vector<tinyobj::material_t>& materials,
    std::string& warn,
    std::string& err,
    std::string& base_dir
  ) {
    std::string filename = fn;
    base_dir = GetBaseDir(filename);
        if (base_dir.empty()) {
            base_dir = ".";
        }
    base_dir += "/";
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(),
                          base_dir.c_str());

   printf("# of vertices  = %d\n", (int)(attrib.vertices.size()) / 3);
   printf("# of normals   = %d\n", (int)(attrib.normals.size()) / 3);
   printf("# of texcoords = %d\n", (int)(attrib.texcoords.size()) / 2);
   printf("# of materials = %d\n", (int)materials.size());
   printf("# of shapes    = %d\n", (int)shapes.size());

}

float get_volume(ApproxMVBB::OOBB oobb){ 

  return (oobb.m_maxPoint(0) - oobb.m_minPoint(0)) * 
         (oobb.m_maxPoint(1) - oobb.m_minPoint(1)) * 
         (oobb.m_maxPoint(2) - oobb.m_minPoint(2));
}

int main(int argc, char** argv)
{
    char * input_filename = argv[1];
    char * output_dir = argv[2];

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    std::string base_dir;
    load_obj(std::string(input_filename), attrib, shapes, materials, warn, err, base_dir);

    unsigned int n_vert = (int)(attrib.vertices.size()) / 3;
    unsigned int nPoints = n_vert;

    std::cout << "Sample " << nPoints << " points in unite cube (coordinates are in world coordinate system `I` ) " << std::endl;
    ApproxMVBB::Matrix3Dyn points(3, nPoints);
    //points.setRandom();

    for (int i = 0; i < nPoints; i++) {
      float x = attrib.vertices[3 * i + 0];
      float y = attrib.vertices[3 * i + 1];
      float z = attrib.vertices[3 * i + 2];
      points.col(i) = ApproxMVBB::Vector3(x, y, z);
    }

    ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,
                                                        0.001,
                                                        500,
                                                        5, /*increasing the grid size decreases speed */
                                                        0,
                                                        5);

    std::cout << "Computed OOBB: " << std::endl
              << "---> lower point in OOBB coordinate system: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB coordinate system: " << oobb.m_maxPoint.transpose() << std::endl
              << "---> coordinate transformation A_IK matrix from OOBB coordinate system `K`  "
                 "to world coordinate system `I` "
              << std::endl
              << oobb.m_q_KI.matrix() << std::endl
              << "---> this is also the rotation matrix R_KI  which turns the "
                 "world coordinate system `I`  into the OOBB coordinate system `K` "
              << std::endl
              << std::endl;

    // To make all points inside the OOBB :
    ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();  // faster to store the transformation matrix first
    auto size                 = points.cols();
    for(unsigned int i = 0; i < size; ++i)
    {
        oobb.unite(A_KI * points.col(i));
    }

    // To make the box have a minimum extent of greater 0.1:
    // see also oobb.expandToMinExtentRelative(...)
    //oobb.expandToMinExtentAbsolute(0.1);

    std::cout << "OOBB with all point included: " << std::endl
              << "---> lower point in OOBB coordinate system: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB coordinate system: " << oobb.m_maxPoint.transpose() << std::endl;

    std::cout << "Volume" << std::endl
              << get_volume(oobb);


    std::ofstream output_file;
    output_file.open (std::string(output_dir) + std::string("/mvbb_out.txt"));
    output_file << oobb.m_q_KI.matrix() << std::endl;
    output_file << oobb.m_minPoint.transpose() << std::endl;
    output_file << oobb.m_maxPoint.transpose() << std::endl;
    output_file << get_volume(oobb) << std::endl;
    output_file.close();

    return 0;
}
