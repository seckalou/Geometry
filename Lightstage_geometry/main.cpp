#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "TriMesh.h"
#include "rapidxml.hpp"

using namespace std;
using namespace cv;
using namespace trimesh;
using namespace rapidxml;

int projectPointsBis(Matx34f trans_mat, Matx33f cam_mat, cv::Vec<float, 5> distCoef, vector<Point3f> in, vector<Point3f> &out){

//    Matx34f cam_trans_mat = cam_mat * trans_mat;
    float fx = cam_mat(0,0);
    float fy = cam_mat(1,1);
    float cx = cam_mat(0,2);
    float cy = cam_mat(1,2);
    float k1 = distCoef(0);
    float k2 = distCoef(1);
    float p1 = distCoef(2);
    float p2 = distCoef(3);
    float k3 = distCoef(4);

    for(int i = 0 ; i < in.size() ; i++){
        Matx41f p3d(in[i].x, in[i].y, in[i].z, 1);
//        Matx31f p2d = cam_trans_mat * p3d;
        Matx31f p2d = trans_mat * p3d;
        Point3f tmp;
        float x = p2d(0,0) / p2d(2,0);
        float y = p2d(1,0) / p2d(2,0);
        float r = sqrt(pow(x,2) + pow(y,2));

        x = x * ( 1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6));
        y = y * ( 1 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6));

        x = cx + x * fx;
        y = cy + y * fy;

        tmp.x = x;
        tmp.y = y;
        tmp.z = p2d(2,0);

        out.push_back(tmp);

    }

    return 1;
}

void normals_to_eye_space(Matx33f rot_mat, const vector<cv::Point3f> &in ,vector<cv::Point3f> &out){

        for(int i = 0 ; i < in.size() ; i++){

            Matx31f n3d(in[i].x, in[i].y, in[i].z);
            n3d = rot_mat * n3d;

            out[i].x = n3d(0,0);
            out[i].y = n3d(0,1);
            out[i].z = n3d(0,2);

        }


}



int main(int argc, char** argv)
{
    if ( argc != 4 )
    {
        printf("usage: cmd <Mesh> <Normal Map> <camera.xml>\n");
        return -1;
    }


    /*************************READING CAMERAS****************************/

    printf("Reading camera parameters...\n");

    map<string, map<string,float> > sensors;
    map<string, map<string,float> > cameras;
    map<string, cv::Point2i> cameras_size;

    ifstream camFile(argv[3]);
    if(camFile.is_open()){

        camFile.seekg(0, camFile.end);
        int camFileLength = camFile.tellg();
        camFile.seekg(0, camFile.beg);

        char* camFileBuff = new char[camFileLength + 1];

        camFile.read(camFileBuff, camFileLength);

        camFileBuff[camFileLength] = 0;

        xml_document<> doc;
        doc.parse<0>(camFileBuff);

        xml_node<> *root_node = doc.first_node("document");
        if(!root_node){
            cerr << "Error on reading XML cameras file" << endl;
            return -1;
        }

        root_node = root_node->first_node("chunk");
        if(!root_node){
            cerr << "Error on reading XML cameras file" << endl;
            return -1;
        }

        xml_node<> *sensors_node = root_node->first_node("sensors");
        if(!sensors_node){
            cerr << "Error on reading XML cameras file...(sensors node missing)" << endl;
            return -1;
        }

        xml_node<> *cameras_node = root_node->first_node("cameras");
        if(!cameras_node){
            cerr << "Error on reading XML cameras file...(cameras node missing)" << endl;
            return -1;
        }

        xml_node<> *node = sensors_node->first_node("sensor");

        while(node){

            float fx, fy, cx, cy, k1, k2, k3;
            xml_attribute<> *att_id = node->first_attribute("id");

            xml_node<> *calib_node = node->first_node("calibration");
            if(!calib_node){
                cerr << "Error on reading XML cameras file" << endl;
                //return -1;
                node = node->next_sibling("sensor");
                continue;
            }

            xml_node<> *param_node = calib_node->first_node("resolution");
            if(param_node){
                cv::Point2i res;
                sscanf(param_node->first_attribute("width")->value(), "%d", &(res.x));
                sscanf(param_node->first_attribute("height")->value(), "%d", &(res.y));
                cameras_size.insert(pair<string, cv::Point2i>(att_id->value(), res));
            }else{
                cerr << "Error on reading XML cameras file...(resolution missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("fx");
            if(param_node){
                sscanf(param_node->value(), "%f", &fx);
            }else{
                cerr << "Error on reading XML cameras file...(fx missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("fy");
            if(param_node){
                sscanf(param_node->value(), "%f", &fy);
            }else{
                cerr << "Error on reading XML cameras file...(fy missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("cx");
            if(param_node){
                sscanf(param_node->value(), "%f", &cx);
            }else{
                cerr << "Error on reading XML cameras file...(cx missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("cy");
            if(param_node){
                sscanf(param_node->value(), "%f", &cy);
            }else{
                cerr << "Error on reading XML cameras file...(cy missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("k1");
            if(param_node){
                sscanf(param_node->value(), "%f", &k1);
            }else{
                cerr << "Error on reading XML cameras file...(k1 missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("k2");
            if(param_node){
                sscanf(param_node->value(), "%f", &k2);
            }else{
                cerr << "Error on reading XML cameras file...(k2 missing)" << endl;
                return -1;
            }

            param_node = calib_node->first_node("k3");
            if(param_node){
                sscanf(param_node->value(), "%f", &k3);
            }else{
                cerr << "Error on reading XML cameras file...(k3 missing)" << endl;
                return -1;
            }

            map<string, float> entry;
            entry.insert(pair<string, float>("fx",fx)); entry.insert(pair<string, float>("fy",fy));
            entry.insert(pair<string, float>("cx",cx)); entry.insert(pair<string, float>("cy",cy));
            entry.insert(pair<string, float>("k1",k1));
            entry.insert(pair<string, float>("k2",k2));
            entry.insert(pair<string, float>("k3",k3));

            string sensor_id (att_id->value());
            sensors.insert(pair<string, map<string, float > > (sensor_id, entry));

            node = node->next_sibling("sensor");
        }

        node = cameras_node->first_node("camera");

        while(node){
            float id, sensor;
            float t0, t1, t2,
                    r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;

            map<string, float> entry;

            xml_attribute<> *att_id = node->first_attribute("id");
            xml_attribute<> *att_sensor_id = node->first_attribute("sensor_id");

            sscanf(att_id->value(), "%f", &id);
            sscanf(att_sensor_id->value(), "%f", &sensor);

            xml_node<> *trans_node = node->first_node("transform");
            if(trans_node){
                sscanf(trans_node->value(), "%f %f %f %f %f %f %f %f %f %f %f %f ",
                       &r00, &r01, &r02, &t0,
                       &r10, &r11, &r12, &t1,
                       &r20, &r21, &r22, &t2);
            }else{
                cerr << "Error on reading XML cameras file" << endl;
                return -1;
            }

            entry.insert(pair<string , float>("sensor", sensor));

            entry.insert(pair<string, float>("r00", r00));
            entry.insert(pair<string, float>("r01", r01));
            entry.insert(pair<string, float>("r02", r02));
            entry.insert(pair<string, float>("r10", r10));
            entry.insert(pair<string, float>("r11", r11));
            entry.insert(pair<string, float>("r12", r12));
            entry.insert(pair<string, float>("r20", r20));
            entry.insert(pair<string, float>("r21", r21));
            entry.insert(pair<string, float>("r22", r22));
            entry.insert(pair<string, float>("t0", t0));
            entry.insert(pair<string, float>("t1", t1));
            entry.insert(pair<string, float>("t2", t2));

            cameras.insert(pair<string, map<string, float> >(att_id->value(), entry));

            node = node->next_sibling("camera");

        }

    }else{
        return -1;
    }

    /*************************READING MESH*****************************/

    TriMesh *themesh = TriMesh::read(argv[1]);


    if(!themesh){
        cout << "A problem occured while reading the mesh" << endl;
        return -1;
    }

    if(themesh->normals.size() == 0){
        themesh->need_normals();
    }

    themesh->need_faces();

//    themesh->faces.clear();


    /************************READING NORMAL MAPS PATHS*****************/

    std::vector<string> normalFiles;
    ifstream normalFilesStream(argv[2]);
    string line;
    while(std::getline(normalFilesStream,line)){
        normalFiles.push_back(line);
//        cout << line << endl;
    }

    /************************PROJECTING ON EACH VIEW*******************/
    Matx33f rot_mat;
    Matx13f trans_vec;
    Matx33f cam_mat(0,0,0,0,0,0,0,0,0);
    cv::Vec<float, 5> distCoef(0,0,0,0,0);

    for(map<string, map<string, float> >::iterator iter = cameras.begin();
        iter != cameras.end(); iter++){

        cout << iter->first << "---------------------------------------------"<< endl << endl;

        int camera_id = atoi(((string)iter->first).c_str());
        if( camera_id != 0 /*&& camera_id != 1 && camera_id != 2 */) continue;


        map<string, float> cam = iter->second;
        char tmp[10];
        sprintf(tmp,"%d",(int)cam.at("sensor"));
        string sensor_id(tmp);
        map<string, float> sen = sensors.at(sensor_id);
        cv::Point2i camera_size = cameras_size.at(sensor_id);

        rot_mat(0,0)=cam.at("r00"); rot_mat(0,1)=cam.at("r01"); rot_mat(0,2)=cam.at("r02");
        rot_mat(1,0)=cam.at("r10"); rot_mat(1,1)=cam.at("r11"); rot_mat(1,2)=cam.at("r12");
        rot_mat(2,0)=cam.at("r20"); rot_mat(2,1)=cam.at("r21"); rot_mat(2,2)=cam.at("r22");

        trans_vec(0,0)= cam.at("t0"); trans_vec(0,1)=cam.at("t1"); trans_vec(0,2)=cam.at("t2");

        Matx34f trans_mat, tmp1;
        hconcat(rot_mat, trans_vec.t(),tmp1);
        Vec4f tmp2(0,0,0,1);
        Matx44f tmp3;
        vconcat(tmp1,tmp2.t(),tmp3);

//        cout << tmp3 << endl << endl;

        tmp3 = tmp3.inv();

//        cout << tmp3 << endl << endl;

        for(int i = 0 ; i < 4 ; i++)
            for(int j = 0 ; j < 3 ; j++)
                trans_mat(j,i) = tmp3(j,i);

//        cout << tmp3 << endl << endl ;
//        cout << trans_mat << endl << endl;

        cam_mat(0,0)=sen.at("fx");cam_mat(1,1)=sen.at("fy");cam_mat(2,2)=1;
        cam_mat(0,2)=sen.at("cx");cam_mat(1,2)=sen.at("cy");

        distCoef(0)=sen.at("k1");
        distCoef(1)=sen.at("k2");
        distCoef(4)=sen.at("k3");


/*
        cout << "INTRINSIC : " << endl<< endl ;

        printf ("k1: %f , k2: %f , K3: %f \n\n", sen.at("k1"),sen.at("k2"),sen.at("k3"));
        cout << "Camera matrix:" << endl << cam_mat << endl << endl;

        cout << "EXTRINSIC :" << endl << endl << trans_mat << endl << endl;

*/


//        distCoef(2)=sen.at("p1"); distCoef(3)=sen.at("p2");

        vector<trimesh::point> vertices = themesh->vertices;
        vector<trimesh::vec> normals = themesh->normals;
        vector<cv::Point3f> points_3D;
        vector<cv::Point3f> normals_3D;

        for(int i = 0 ; i < vertices.size() ; i++){
            trimesh::point v = vertices[i];
            trimesh::vec vv = normals[i];
            Point3f p(v[0],v[1],v[2]);
            Point3f n(vv[0], vv[1], vv[2]);

            points_3D.push_back(p);
            normals_3D.push_back(n);

        }

        vector<cv::Point3f> points_2D;
//        projectPoints(points_3D, vec_rot, trans_vec, cam_mat, distCoef,points_2D);

        projectPointsBis(trans_mat, cam_mat, distCoef , points_3D, points_2D);

        normals_to_eye_space(rot_mat.inv(), normals_3D,normals_3D);

        cv::Point3f up_left = points_2D[0];
        cv::Point3f down_right = points_2D[0];
        float max_z = points_3D[0].z;
        float min_z = points_3D[0].z;

        /**************************READING THE NORMAL MAP FOR THIS VIEW**********************/

        string normalFile = normalFiles.at(atoi(((string)iter->first).c_str()));
//        Mat normalMap = imread(normalFile,
//                               CV_LOAD_IMAGE_COLOR);

        IplImage* normalMap = cvLoadImage(normalFile.c_str());

//        cout << normalMap->width << " , " << normalMap->height << " , " << normalMap->nChannels << endl;

//        themesh->colors.resize(points_3D.size());

        for(int i = 0 ; i < points_2D.size() ; i++){

            cv::Point3f p = points_2D.at(i);


//            if(up_left.x > p.x ) up_left.x = p.x;
//            if(up_left.y < p.y ) up_left.y = p.y;
//            if(down_right.x < p.x) down_right.x = p.x;
//            if(down_right.y > p.y) down_right.y = p.y;

            float z  = points_2D[i].z;

            if(max_z < z) max_z = z;
            if(min_z > z) min_z = z;

        }

//        float img_scale = 1;
//        int w = (down_right.x - up_left.x)*img_scale;
//        int h = (up_left.y - down_right.y)*img_scale;

        Mat img;
//        img.create(h + 3,w + 3,CV_8U);
        img.create(camera_size.y + 3, camera_size.x + 3,CV_8U);


//        cout << up_left << endl;
//        cout << down_right << endl;
//        cout << w << " , " << h << endl;
//        cout << img.cols << " , " << img.rows << endl;
//        cout << min_z << " , " << max_z << endl;

        for(int i = 0 ; i < points_3D.size() ; i++){
//            int x = floor(points_2D[i].x - up_left.x) * img_scale + 1;
//            int y = floor(points_2D[i].y - up_left.y) * img_scale + 1;
//            int x = floor(-abs(points_2D[i].x + cam_mat(0,2))) * img_scale + 1;
//            int y = floor(-abs(points_2D[i].y + cam_mat(1,2))) * img_scale + 1;
            int x = round(points_2D[i].x);
            int y = round(points_2D[i].y);


            if(x < 0 || x >= normalMap->width) continue;
            if(y < 0 || y >= normalMap->height) continue;
            if(normals_3D[i].dot(Point3f(0,0,1)) > 0) continue;

//            cout << "x: " << x << " , y: " << y<< endl;

            int pos = x * normalMap->nChannels + y * normalMap->width * normalMap->nChannels;

            unsigned char b = normalMap->imageData[pos];
            unsigned char g = normalMap->imageData[pos + 1];
            unsigned char r = normalMap->imageData[pos + 2];

//            themesh->colors.at( i ) =  Color( (float)r / 255 , (float)g / 255 , (float)b / 255 );

            float nx = ((float)r / 255) * 2 - 1;
            float ny = ((float)g / 255) * 2 - 1;
            float nz = ((float)b / 255) * 2 - 1;

            Matx31f n3d(nx, ny, -nz);
            n3d = rot_mat * n3d;

            nx = n3d(0,0);
            ny = n3d(0,1);
            nz = n3d(0,2);

            float length = sqrt(pow(nx,2) + pow(ny, 2) + pow(nz, 2));

            themesh->normals.at(i)[0] = nx / length;
            themesh->normals.at(i)[1] = ny / length;
            themesh->normals.at(i)[2] = nz / length;

            float z = points_2D[i].z;

            img.row(y).col(x) = z * 255 / (max_z - min_z);
        }


        char outputfile[100];
        sprintf(outputfile,"norm:/home/alou/Tmp/test_%s.bmp", ((string)iter->first).c_str());
        imwrite(outputfile, img);

    }


    themesh->write("norm:/home/alou/Tmp/mesh.ply");



    return 0;

}

