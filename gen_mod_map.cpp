//
// Created by lzw on 2021/11/18.
//

#include <iostream>
#include"../mod_map/mod_map.h"
//#include"../mod_map/test_case_lib.h"
void test_case_1();
void test_case_2();
void test_rotate_rays();
void draw_fp_loc_trajectory();
void render_2d_depth_test_1();
int main()
{

    render_2d_depth_test_1();
    //lzw_mod::draw_other_trajectory();
    //draw_fp_loc_trajectory();
    //test_case_2();




}


void test_rotate_rays()
{   lzw_mod::mod_map mod_map;
    Eigen::Matrix4d T,ori;
    std::vector<Eigen::Matrix4d,
            Eigen::aligned_allocator<Eigen::Matrix4d>> v_abs_poses;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds;
    v_abs_poses.push_back(T);
    v_abs_poses.push_back(ori);
    T<<std::cos(M_PI/2),-std::sin(M_PI/2),0,ORIGIN_POS_X,
            std::sin(M_PI/2),std::cos(M_PI/2),0,ORIGIN_POS_Y,
            0,0,1,0,
            0,0,0,1;

    ori<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    pcl::PointXYZ start_p(T(0,3),T(1,3),0);
    //width:5000  height:3000

    //different ray are control by angle

    //it_angle=0,1,2,...,8
    double sample_rate=10; //degree
    double start_angle=-45;
    std::vector<lzw_mod::abc_2d_line> v_rays;
    for(uint it_angle=0;it_angle<9;it_angle++) {
        double current_angle = start_angle + sample_rate * it_angle;
        double current_rad = current_angle * M_PI / 180;

        pcl::PointXYZ ray_end_p(start_p.x + 1000 * std::cos(current_rad),
                                start_p.y + 1000 * std::sin(current_rad), 0);


        Eigen::Matrix<double, 3, 1> egm_ray_end_p;
        egm_ray_end_p << ray_end_p.x, ray_end_p.y, 0;
        egm_ray_end_p = T.block(0, 0, 3, 3) * egm_ray_end_p;
        std::cout << "T: " << std::endl;
        std::cout << T.block(0, 0, 3, 3) << endl;

        //
        /*Eigen::Matrix<double,3,3> egm_test;
        egm_test<<std::cos(M_PI/2),-std::sin(M_PI/2),0,
                std::sin(M_PI/2),std::cos(M_PI/2),0,
                0,0,1;

        egm_ray_end_p=egm_test*egm_ray_end_p;*/
        //

        pcl::PointXYZ trans_ray_end_p(egm_ray_end_p(0), egm_ray_end_p(1), 0);
        lzw_mod::abc_2d_line ray(start_p, trans_ray_end_p);


        v_rays.push_back(ray);
        pcl::PointXYZ cloest_point;
        double distance = 999999999999999;
        std::cout << ray.start_ << " " << ray.end_ << endl;
    }
    //draing points, lines,pose ,and map itself
    //v_draw_flags contrals whether draw the corrsponding matter
    //v[0] <==> v_l
    //v[1] <==> poses
    //v[2] <==> v_p_pcds
    //v[3] <==> map_itself
    //v[4] <==> whether linking the poses with line
    vector<double> v_draw_flags_2={1,1,0,1,0};
    mod_map.draw_multi_trajectory(v_rays,v_abs_poses,v_p_pcds,100,v_draw_flags_2);
}

void render_2d_depth_test_1()
{
    lzw_mod::mod_map mod_map;
    std::vector<Eigen::Matrix4d,
            Eigen::aligned_allocator<Eigen::Matrix4d>> v_abs_poses;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds;
    Eigen::Matrix3d egm_cam_K;

    Eigen::Matrix4d egm_w_ori,egm_f_ori,egm_f2,egm_f3,egm_f4;//egm means eigen matrix
    //world coor of PCL viewer
    egm_w_ori<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    //world coor of floor plan
    egm_f_ori<<1,0,0,  ORIGIN_POS_X,
            0,1,0,ORIGIN_POS_Y,
            0,0,1,0,
            0,0,0,1;
    //rot  around z about 90 degree
    egm_f2<<std::cos(M_PI/2),-std::sin(M_PI/2),0,ORIGIN_POS_X,
            std::sin(M_PI/2),std::cos(M_PI/2),0,ORIGIN_POS_Y+400,
            0,0,1,0,
            0,0,0,1;

    egm_f3<<std::cos(M_PI/2),-std::sin(M_PI/2),0,ORIGIN_POS_X-2000,
            std::sin(M_PI/2),std::cos(M_PI/2),0,ORIGIN_POS_Y-2000,
            0,0,1,0,
            0,0,0,1;
    egm_f4<<std::cos(M_PI/6),-std::sin(M_PI/6),0,ORIGIN_POS_X,
            std::sin(M_PI/6),std::cos(M_PI/6),0,ORIGIN_POS_Y-2000,
            0,0,1,0,
            0,0,0,1;
    v_abs_poses.push_back(egm_w_ori);
    v_abs_poses.push_back(egm_f_ori);
    v_abs_poses.push_back(egm_f2);
    v_abs_poses.push_back(egm_f3);
    v_abs_poses.push_back(egm_f4);


    mod_map.render_2d_depth(v_abs_poses,v_p_pcds,egm_cam_K);
}
void test_case_1()
{
    lzw_mod::mod_map mod_map;
    // mod_map.draw_multi_trajectory();
    //x_min , x_max, y_min, y_max;
    double x1,x2,y1,y2;
    x2=  mod_map.opt_map_._boundary.x_max;
    x1=mod_map.opt_map_._boundary.x_min;
    y2=mod_map.opt_map_._boundary.y_max;
    y1=mod_map.opt_map_._boundary.y_min;

    cout<<"x_max: "<<mod_map.opt_map_._boundary.x_max<<endl;
    cout<<"x_min: "<<mod_map.opt_map_._boundary.x_min<<endl;
    cout<<"y_max: "<<mod_map.opt_map_._boundary.y_max<<endl;
    cout<<"y_min: "<<mod_map.opt_map_._boundary.y_min<<endl;
    lzw_mod::abc_2d_line l1(pcl::PointXYZ(x1,y1,0),pcl::PointXYZ(x2,y1,0)),
            l2(pcl::PointXYZ(x1,y1,0),pcl::PointXYZ(x1,y2,0)),
            l3(pcl::PointXYZ(x1,y2,0),pcl::PointXYZ(x2,y2,0)),
            l4(pcl::PointXYZ(x2,y1,0),pcl::PointXYZ(x2,y2,0));
    std::vector<lzw_mod::abc_2d_line> v_ls;
    v_ls.push_back(l1); v_ls.push_back(l2); v_ls.push_back(l3); v_ls.push_back(l4);

    pcl::PointXYZ start_p(ORIGIN_POS_X,ORIGIN_POS_Y,0);
    //width:5000  height:3000

    lzw_mod::abc_2d_line ray(start_p,pcl::PointXYZ(start_p.x+100,start_p.y+100,0));
    v_ls.push_back(ray);

    std::vector<Eigen::Matrix4d,
            Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    Eigen::Matrix4d egm_w_ori,egm_f_ori,egm_f2;//egm means eigen matrix
    //world coor of PCL viewer
    egm_w_ori<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    //world coor of floor plan
    egm_f_ori<<1,0,0,38935,
            0,1,0,20800,
            0,0,1,0,
            0,0,0,1;

    egm_f2<<std::cos(M_PI/2),-std::sin(M_PI/2),0,38935,
            std::sin(M_PI/2),std::cos(M_PI/2),0,20800+400,
            0,0,1,0,
            0,0,0,1;
    poses.push_back(egm_f_ori);
    poses.push_back(egm_f2);
    poses.push_back(egm_w_ori);

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds;


    //test intersection
    pcl::PointCloud<pcl::PointXYZ>::Ptr  v_intersections(new  pcl::PointCloud<pcl::PointXYZ>);
    for (uint it_s = 0; it_s < mod_map.opt_map_._shape_list.size(); it_s++) {  //double line_color=(it_tra+1)*color_div;
        double axis_len = 0.15;

        Shape *p_s = mod_map.opt_map_._shape_list[it_s];
        Line *p_l = dynamic_cast<Line *>(p_s);
        pcl::PointXYZ start(p_l->_start_point.x,
                            p_l->_start_point.y,
                            0);
        pcl::PointXYZ end(p_l->_end_point.x,
                          p_l->_end_point.y,
                          0);

        if(lzw_mod::mod_map::is_intersection(ray.start_,ray.end_,start,end))
        {

            v_intersections->points.push_back(lzw_mod::mod_map::
                                              calculate_intersection(ray.start_,ray.end_,start,end));
        }

    }

    v_p_pcds.push_back(v_intersections);
    //v_draw_flags contrals whether draw the corrsponding matter
    //v[0] <==> v_l (the lines that you want to add)
    //v[1] <==> poses
    //v[2] <==> v_p_pcds
    //v[3] <==> map_itself
    //v[4] <==> whether linking the poses with line

    std::vector<double> v_draw_flags= {1,1,1,1,0};
    mod_map.draw_multi_trajectory(v_ls,poses,v_p_pcds,1000,v_draw_flags);

}

void draw_fp_loc_trajectory()
{
   // std::string s_base_path_trajec="/home/lzw/Desktop/2021_10_project/code/FP-Loc/FP-Loc/output/"
                              //"previous_re/results/TC.B1.Corridor/Loop/Fast/aligned/";
   // std::string s_path_postprefix="FP-LOC.txt";
   //when mod string in this place, plz remember that mod ostream in lzw_mod::read_txt
   std::string s_path="/home/lzw/Desktop/2021_10_project/code/FP-Loc/"
                      "FP-Loc/output/previous_re/results/BME.RM4307.Unfurnished_Room/Big_Loop/Fast/FP-Loc.txt";

    lzw_mod::mod_map mod_map;
    Eigen::Matrix4d egm_w_ori,egm_f_ori;
    egm_w_ori<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    //world coor of floor plan
    egm_f_ori<<1,0,0,  ORIGIN_POS_X,
            0,1,0,ORIGIN_POS_Y,
            0,0,1,0,
            0,0,0,1;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> v_abs_pose;
    v_abs_pose.push_back(egm_w_ori);
    v_abs_pose.push_back(egm_f_ori);
    lzw_mod::read_txt(s_path,v_abs_pose);

    //begin is v_abs_pose[0]
    //so  begin+1 is v_abs_pose[1]
    //transform the v_abs_pose to  floor plan coor
    for(auto it_p=v_abs_pose.begin()+2;it_p!=v_abs_pose.end();it_p++)
    {
        Eigen::Matrix4d T_fw;//from world to floor plan
        double c_v=std::cos(-ORIGIN_ORIENTATION);
        double s_v=std::sin(-ORIGIN_ORIENTATION);
        T_fw<<c_v,-s_v,0,ORIGIN_POS_X,
                s_v,c_v,0,ORIGIN_POS_Y,
                0,0,1,0,
                0,0,0,1;
        /*T_fw<<1,0,0,ORIGIN_POS_X,
                0,1,0,ORIGIN_POS_Y,
                0,0,1,0,
                0,0,0,1;*/
        Eigen::Vector4d abs_t;
        //std::distance(p,q) p must in front of q
        cout<<"times: "<<std::distance(v_abs_pose.begin(),it_p)<<std::endl;
        std::cout<<"abs pose: "<<std::endl<<*it_p<<std::endl;
        abs_t=(*it_p).block(0,3,4,1);

        std::cout<<"abs t: "<<std::endl<<abs_t<<std::endl;
        abs_t=T_fw*abs_t;
        (*it_p).block(0,3,4,1)=abs_t;
        std::cout<<"trans abs pose: "<<std::endl<<*it_p<<std::endl;
    }


    //draing points, lines,pose ,and map itself
    //v_draw_flags contrals whether draw the corrsponding matter
    //v[0] <==> v_l
    //v[1] <==> poses
    //v[2] <==> v_p_pcds
    //v[3] <==> map_itself
    //v[4] <==> whether linking the poses with line
    vector<double> v_draw_flags_2={1,1,1,1,1};
    // gen bounding box
    double x1,x2,y1,y2;
    x2=  mod_map.opt_map_._boundary.x_max;
    x1=mod_map.opt_map_._boundary.x_min;
    y2=mod_map.opt_map_._boundary.y_max;
    y1=mod_map.opt_map_._boundary.y_min;

    cout<<"x_max: "<<mod_map.opt_map_._boundary.x_max<<endl;
    cout<<"x_min: "<<mod_map.opt_map_._boundary.x_min<<endl;
    cout<<"y_max: "<<mod_map.opt_map_._boundary.y_max<<endl;
    cout<<"y_min: "<<mod_map.opt_map_._boundary.y_min<<endl;
    lzw_mod::abc_2d_line l1(pcl::PointXYZ(x1,y1,0),pcl::PointXYZ(x2,y1,0)),
            l2(pcl::PointXYZ(x1,y1,0),pcl::PointXYZ(x1,y2,0)),
            l3(pcl::PointXYZ(x1,y2,0),pcl::PointXYZ(x2,y2,0)),
            l4(pcl::PointXYZ(x2,y1,0),pcl::PointXYZ(x2,y2,0));
    std::vector<lzw_mod::abc_2d_line> v_ls;
    v_ls.push_back(l1); v_ls.push_back(l2); v_ls.push_back(l3); v_ls.push_back(l4);
    //
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  v_p_marker(new  pcl::PointCloud<pcl::PointXYZ>);
    v_p_marker->push_back(pcl::PointXYZ(30,0,0));
    v_p_marker->push_back(pcl::PointXYZ(0,30,0));
    v_p_marker->push_back(pcl::PointXYZ(0,0,30));
    v_p_pcds.push_back(v_p_marker);




    mod_map.draw_multi_trajectory(v_ls,v_abs_pose,v_p_pcds,30,v_draw_flags_2);

}