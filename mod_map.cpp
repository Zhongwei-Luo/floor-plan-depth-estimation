//
// Created by lzw on 2021/11/18.
//

#include "mod_map.h"

namespace  lzw_mod
{
     mod_map::mod_map()
    {

        cad_cloud_.reset(new pcl::PointCloud<PointType>());
         vis_map_ = CadMap(VIS_MAP_ROUTE, SCALE, ORIGIN_POS_X, ORIGIN_POS_Y,
                           ORIGIN_ORIENTATION, CAD_MAP_NON_WALL_WEIGHT, CAD_MAP_AREA_SIZE);
        vis_map_.Convert2PtCloud(cad_cloud_, CAD_CLOUD_SAMPLE_RATE);
        vis_annf_ = ANNF(& vis_map_, ANNF_MAX_DEPTH);
        vis_annf_.WriteIntoCSV(OUTPUT_DIRECTORY + "vis.csv");
        opt_map_ = CadMap(OPT_MAP_ROUTE, SCALE, ORIGIN_POS_X, ORIGIN_POS_Y, ORIGIN_ORIENTATION, CAD_MAP_NON_WALL_WEIGHT, CAD_MAP_AREA_SIZE);
        // opt_map.Convert2PtCloud(cad_cloud, CAD_CLOUD_SAMPLE_RATE);
        annf_ = ANNF(& opt_map_, ANNF_MAX_DEPTH);

    }
//dynamic_cast<entPlayer*>
   /* void mod_map::draw_multi_trajectory(std::vector<Line *> v_shapes)
    {
        pcl::visualization::PCLVisualizer vis_er;
        vis_er.setBackgroundColor(200, 200, 200);
        int line_num=0;
        for(uint it_s=0;it_s<v_shapes.size();it_s++)
        {  //double line_color=(it_tra+1)*color_div;
            double axis_len = 0.15;
            pcl::PointXYZ start(v_shapes[it_s]->_start_point.x,
                                    v_shapes[it_s]->_start_point.y,
                                    0);
            pcl::PointXYZ end(v_shapes[it_s]->_end_point.x,
                                v_shapes[it_s]->_end_point.y,
                                0);
                vis_er.addLine(start, end, 255, 0, 0, std::to_string (it_s)   );
        }
        vis_er.spin();
    }*/

//draw the map itself using addline()
   void mod_map::draw_multi_trajectory()
    {

        pcl::visualization::PCLVisualizer vis_er;
        vis_er.setBackgroundColor(200, 200, 200);
        int line_num=0;
        for(uint it_s=0;it_s<vis_map_._shape_list.size();it_s++)
        {  //double line_color=(it_tra+1)*color_div;
            double axis_len = 0.15;


            Shape* p_s=vis_map_._shape_list[it_s];
            Line* p_l= dynamic_cast<Line*>(p_s);

            pcl::PointXYZ start(p_l->_start_point.x,
                                p_l->_start_point.y,
                                0);
            pcl::PointXYZ end(p_l->_end_point.x,
                              p_l->_end_point.y,
                              0);

            vis_er.addLine(start, end, 255, 0, 0, std::to_string (it_s)   );

        }

        vis_er.spin();
    }



    //draing points, lines,pose ,and map itself
    //v_draw_flags contrals whether draw the corrsponding matter
    //v[0] <==> v_l
    //v[1] <==> poses
    //v[2] <==> v_p_pcds
    //v[3] <==> map_itself
    //v[4] <==> whether linking the poses with line

    void mod_map:: draw_multi_trajectory(std::vector<abc_2d_line> &v_l,
                               std::vector<Eigen::Matrix4d,
                               Eigen::aligned_allocator<Eigen::Matrix4d>> poses,
                               std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds,
                               double axis_size,
                               vector<double> v_draw_flags)
    {

        pcl::visualization::PCLVisualizer vis_er;
        vis_er.setBackgroundColor(200, 200, 200);

        if(v_draw_flags[3]==1) {
            for (uint it_s = 0; it_s < opt_map_._shape_list.size(); it_s++) {  //double line_color=(it_tra+1)*color_div;
                double axis_len = 0.15;

                Shape *p_s = opt_map_._shape_list[it_s];
                Line *p_l = dynamic_cast<Line *>(p_s);
                pcl::PointXYZ start(p_l->_start_point.x,
                                    p_l->_start_point.y,
                                    0);
                pcl::PointXYZ end(p_l->_end_point.x,
                                  p_l->_end_point.y,
                                  0);
                vis_er.addLine(start, end, 255, 0, 0, "map_line"+std::to_string(it_s));
            }
        }



        int line_num=0;
        if((v_l.size()!=0)&&(v_draw_flags[0]==1)) {
            for (uint it_s = 0; it_s < v_l.size(); it_s++) {  //double line_color=(it_tra+1)*color_div;
                double axis_len = 0.15;
                vis_er.addLine(v_l[it_s].start_, v_l[it_s].end_, 0, 0, 255, "other_line"+std::to_string(it_s));
            }
        }
        //

        pcl::PointXYZ last_O_pos(0, 0, 0);
        double axis_len = axis_size;
        if((poses.size()!=0)&&(v_draw_flags[1]=1))
        {   for (int i = 0; i < poses.size(); i++)
            {
                pcl::PointXYZ O_pose;
                //每帧位姿的原点坐标只由变换矩阵中的平移向量得到
                O_pose.x = poses[i](0, 3);
                O_pose.y = poses[i](1, 3);
                O_pose.z = poses[i](2, 3);
                if ((i > 0)&&(v_draw_flags[4]==1))
                    vis_er.addLine(last_O_pos, O_pose, 0, 0, 0, "trac_" + std::to_string(i));

                pcl::PointXYZ X;

                Eigen::MatrixXd eg_x(4, 1);
                eg_x << axis_len, 0, 0, 1;
                Eigen::MatrixXd Xw(4, 1);
                Xw = poses[i] * (eg_x);
                X.x = Xw(0, 0);
                X.y = Xw(1, 0);
                X.z = Xw(2, 0);
                //drawing x axis,x axis->red
                vis_er.addLine(O_pose, X, 255, 0, 0, "X_" + std::to_string(i));

                pcl::PointXYZ Y;
                Eigen::MatrixXd eg_y(4, 1);
                eg_y << 0, axis_len, 0, 1;
                Eigen::MatrixXd Yw(4, 1);
                Yw = poses[i] * (eg_y);
                Y.x = Yw(0, 0);
                Y.y = Yw(1, 0);
                Y.z = Yw(2, 0);
                //drawing y axis,y axis is green
                vis_er.addLine(O_pose, Y, 0, 255, 0, "Y_" + std::to_string(i));

                pcl::PointXYZ Z;
                Eigen::MatrixXd eg_z(4, 1);
                eg_z << 0, 0, axis_len, 1;
                Eigen::MatrixXd Zw(4, 1);
                Zw = poses[i] * (eg_z);
                Z.x = Zw(0, 0);
                Z.y = Zw(1, 0);
                Z.z = Zw(2, 0);
                //drwaing z axis, z axis ->blue

                vis_er.addLine(O_pose, Z, 0, 0, 255, "Z_" + std::to_string(i));

                last_O_pos = O_pose;
            }}


        if((v_p_pcds.size()!=0)&&(v_draw_flags[2]==1))
        {
            for(uint it_p=0;it_p<=v_p_pcds.size()-1;it_p++ )
            {  double r_c,g_c,b_c;
                r_c=1;b_c=1;
                //g_c=254/v_p_pcds.size();
                //g_c=g_c*it_p;
                g_c=255;
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(v_p_pcds[it_p], r_c,g_c,b_c);
                vis_er.addPointCloud<pcl::PointXYZ>(v_p_pcds[it_p], single_color, "cloud_"+std::to_string(it_p));
                vis_er.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_"+std::to_string(it_p));
            }

        }

        //spin()
        vis_er.spin();
    };



    pcl::PointXYZ mod_map:: calculate_intersection(pcl::PointXYZ &a,pcl::PointXYZ &b,pcl::PointXYZ &c,pcl::PointXYZ &d)
    {
        abc_2d_line m(a,b),n(c,d);
        /* m.a0_=a.y-b.y;
         m.b0_=b.x-a.x;
         m.c0_=a.x*b.y-a.y*b.x;

         n.a0_=c.y-d.y;
         n.b0_=d.x-c.x;
         n.c0_=c.x*d.y-c.y*d.x;*/
        const double p=1e-10;
        double x0,p0;
        double k1,k2,b1,b2;
        double D;

        D=m.a0_*n.b0_-n.a0_*m.b0_;
        x0=(m.b0_*n.c0_-n.b0_*m.c0_)/D;
        p0=(n.a0_*m.c0_-m.a0_*n.c0_)/D;

        pcl::PointXYZ intersection(x0,p0,0);
        return intersection;


    }

    int mod_map:: is_intersection(pcl::PointXYZ a,pcl::PointXYZ b,pcl::PointXYZ c,pcl::PointXYZ d)
    {
        double p1,p2,p3,p4;
        const double p=1e-10;
        p1=(d.x-c.x)*(d.y-a.y)-(d.y-c.y)*(d.x-a.x);//DCxDA
        p2=(d.x-c.x)*(d.y-b.y)-(d.y-c.y)*(d.x-b.x);//DCxDB
        p3=(b.x-a.x)*(b.y-d.y)-(b.y-a.y)*(b.x-d.x);//BAxBD
        p4=(b.x-a.x)*(b.y-c.y)-(b.y-a.y)*(b.x-c.x);//BAxBC
        if(p1*p2<=p&&p3*p4<=p)//double判断有精度问题 注意
            return 1;
        return 0;
    }

    void  mod_map::render_2d_depth(std::vector<Eigen::Matrix4d,
            Eigen::aligned_allocator<Eigen::Matrix4d>> v_abs_poses,
                         std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >& v_p_pcds,
                         Eigen::Matrix3d egm_cam_K)
    {
        double bx1,bx2,by1,by2;
        bx2=opt_map_._boundary.x_max;
        bx1=opt_map_._boundary.x_min;

        by2=opt_map_._boundary.y_max;
        by1=opt_map_._boundary.y_min;

        //對角線

        double len_dia=std::sqrt(abs(by2-by1)*abs(by2-by1)+abs(bx1-bx2)*abs(bx1-bx2));

        for(uint it_p=0;it_p<v_abs_poses.size();it_p++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr p_pcd_intersections (new   pcl::PointCloud<pcl::PointXYZ>);
            //render each single depth  at location
            //to do: fov is depend on egm_cam_K
            //for testing,we just create a fov independ of egm_cam_k
            //create a list of ray,staring from
            pcl::PointXYZ start_p(v_abs_poses[it_p](0,3),v_abs_poses[it_p](1,3),0);
            //width:5000  height:3000

            //different ray are control by angle

            //it_angle=0,1,2,...,8
            double sample_rate=2; //degree
            double start_angle=-45;
            std::vector<abc_2d_line> v_rays;
            for(uint it_angle=0;it_angle<45;it_angle++)
            {   double current_angle=start_angle+sample_rate*it_angle;
                double current_rad=current_angle*M_PI/180;


                //warning:
                //whether use test value:500 or len_dia
                /*pcl::PointXYZ ray_end_p(start_p.x+ len_dia*std::cos(current_rad),
                             start_p.y+len_dia*std::sin(current_rad),0);
*/
               pcl::PointXYZ ray_end_p( len_dia*std::cos(current_rad),
                                        len_dia*std::sin(current_rad),0);
                Eigen::Matrix<double,4,1> egm_ray_end_p;
                egm_ray_end_p<<ray_end_p.x,ray_end_p.y,0,1;
                cout<<"before rotated egm ray:"<<egm_ray_end_p<<endl;
                cout<<"before rotated pcl p: "<<ray_end_p.x<<" "<<ray_end_p.y<<std::endl;

                egm_ray_end_p=v_abs_poses[it_p]*egm_ray_end_p;
                std::cout<<"T: "<<std::endl;
                std::cout<<v_abs_poses[it_p]<<endl;
                cout<<"after rotated egm ray:"<<egm_ray_end_p<<endl;
                //
                /*Eigen::Matrix<double,3,3> egm_test;
                egm_test<<std::cos(M_PI/2),-std::sin(M_PI/2),0,
                        std::sin(M_PI/2),std::cos(M_PI/2),0,
                        0,0,1;

                egm_ray_end_p=egm_test*egm_ray_end_p;*/
                //

                pcl::PointXYZ trans_ray_end_p(egm_ray_end_p(0),egm_ray_end_p(1),0);

                //warning:
                //whether use trans ray or non-tran ray
                abc_2d_line ray(start_p,trans_ray_end_p);
                //abc_2d_line ray(start_p,ray_end_p);
                v_rays.push_back(ray);
                pcl::PointXYZ cloest_point;
                double distance=999999999999999;
                //std::cout <<ray.start_<<" "<<ray.end_<<endl;
                //each ray need to calculate intersection for all map lines;
                for (uint it_s = 0; it_s < opt_map_._shape_list.size(); it_s++)
                {  //double line_color=(it_tra+1)*color_div;
                    double axis_len = 0.15;

                    Shape *p_s = opt_map_._shape_list[it_s];
                    Line *p_l = dynamic_cast<Line *>(p_s);
                    pcl::PointXYZ one_l_start(p_l->_start_point.x,
                                              p_l->_start_point.y,
                                              0);
                    pcl::PointXYZ one_l_end(p_l->_end_point.x,
                                            p_l->_end_point.y,
                                            0);
                    pcl::PointXYZ inter_p;
                    if(lzw_mod::mod_map::is_intersection(ray.start_,ray.end_,one_l_start,one_l_end))
                    {   inter_p= lzw_mod::mod_map::calculate_intersection(ray.start_,ray.end_,
                                                       one_l_start,one_l_end);
                        double tem_dis=std::pow(inter_p.x-ray.start_.x,2)+std::pow(inter_p.y-ray.start_.y,2);
                        tem_dis=std::sqrt(tem_dis);
                        if(tem_dis<distance)
                        {cloest_point=inter_p;
                            distance=tem_dis;
                        //cout<<"dis: "<<tem_dis<<endl;
                        //cout<<"cloest_point: "<<cloest_point.x<<" "<<cloest_point.y<<endl;
                        }
                    }
                }
                p_pcd_intersections->push_back(cloest_point);
            }
            //for vis: the following lines could be delete
            //l1~l4 are bounding box
            lzw_mod::abc_2d_line l1(pcl::PointXYZ(bx1,by1,0),pcl::PointXYZ(bx2,by1,0)),
                    l2(pcl::PointXYZ(bx1,by1,0),pcl::PointXYZ(bx1,by2,0)),
                    l3(pcl::PointXYZ(bx1,by2,0),pcl::PointXYZ(bx2,by2,0)),
                    l4(pcl::PointXYZ(bx2,by1,0),pcl::PointXYZ(bx2,by2,0));

            v_rays.push_back(l1); v_rays.push_back(l2); v_rays.push_back(l3); v_rays.push_back(l4);
            v_p_pcds.push_back(p_pcd_intersections);
            //v_draw_flags contrals whether draw the corrsponding matter
            //v[0] <==> v_l
            //v[1] <==> poses
            //v[2] <==> v_p_pcds
            //v[3] <==> map_itself
            //v[4] <==> whether linking the poses with line
            vector<double> v_draw_flags={1,1,1,1,0};
            draw_multi_trajectory(v_rays,v_abs_poses,v_p_pcds,1000,v_draw_flags);
            vector<double> v_draw_flags_2={0,1,1,0,0};
            draw_multi_trajectory(v_rays,v_abs_poses,v_p_pcds,100,v_draw_flags_2);
        }

    }

    void read_txt_no_mod(std:: string file,
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_abs_pose)
    {
        ifstream infile;
        //std::ofstream  obj_outfile("/home/lzw/Desktop/2021_10_project/code/FP-Loc/FP-Loc/output/previous_re/"
        //                           "results/TC.B1.Corridor/Loop/Fast/aligned/mod_fp_loc.txt");


        infile.open(file.data());   //将文件流对象与文件连接起来
        assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
        vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
        ifstream fin(file);
        if (!fin) {
            cout << "cannot find trajectory file at " << file << endl;
        }
        int out_put_count = 0;
        while (!fin.eof()) {
            double time, tx, ty, tz, qx, qy, qz, qw;
            fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;


            //just using the ori quaterniond
            //------------------------------------

            Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
            poses.push_back(Twr);
            Eigen::Matrix4d egm_T_wr;
            egm_T_wr = Eigen::Matrix4d::Zero();
            egm_T_wr.block(0, 0, 3, 3) = Twr.rotation();
            egm_T_wr.block(0, 3, 3, 1) = Twr.translation() / SCALE;
            egm_T_wr(3, 3) = 1;
            v_abs_pose.push_back(egm_T_wr);


        }
    }
    void read_txt(std:: string file,
                  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_abs_pose
                  )
    {
        ifstream infile;
        //std::ofstream  obj_outfile("/home/lzw/Desktop/2021_10_project/code/FP-Loc/FP-Loc/output/previous_re/"
        //                           "results/TC.B1.Corridor/Loop/Fast/aligned/mod_fp_loc.txt");

        std::ofstream obj_outfile("/home/lzw/Desktop/2021_10_project/code/FP-Loc/FP-Loc/output/previous_re/"
                                  "results/BME.RM4307.Unfurnished_Room/Big_Loop/Fast/mod_fp_loc.txt");
        infile.open(file.data());   //将文件流对象与文件连接起来
        assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
        vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
        ifstream fin(file);
        if (!fin)
        {
            cout << "cannot find trajectory file at " << file << endl;
        }
        int out_put_count=0;
        while (!fin.eof())
        {
            double time, tx, ty, tz, qx, qy, qz, qw;
            fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;


            //just using the ori quaterniond
            //------------------------------------
            /*
             Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
            poses.push_back(Twr);
            Eigen::Matrix4d egm_T_wr;
            egm_T_wr=Eigen::Matrix4d::Zero();
            egm_T_wr.block(0,0,3,3)=Twr.rotation();
            egm_T_wr.block(0,3,3,1)=Twr.translation()/SCALE;
            egm_T_wr(3,3)=1;
            v_abs_pose.push_back(egm_T_wr);
            */
            //-------------------------------------

            //change the ori quaterniond to euler angle, let
            //-------------------------------------
            Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
            Eigen::Vector3d eulerAngle=Eigen::Quaterniond(qw, qx, qy, qz).matrix().eulerAngles(2,1,0);
            //reverse R of RPY
            std::cout<<"euler angle :"<<eulerAngle<<std::endl;
            eulerAngle(0)=-eulerAngle(0);
            std::cout<<"after reverse euler angle :"<<eulerAngle<<std::endl;

            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
            Eigen::Matrix3d rotation_matrix;rotation_matrix=yawAngle*pitchAngle*rollAngle;

            Eigen::Matrix4d egm_T_wr;
            egm_T_wr=Eigen::Matrix4d::Zero();
            egm_T_wr.block(0,0,3,3)=rotation_matrix;
            egm_T_wr.block(0,3,3,1)=Twr.translation()/SCALE;
            egm_T_wr(3,3)=1;
            v_abs_pose.push_back(egm_T_wr);
            //gen mod  trajectory
            Eigen::Quaterniond new_quaternion(rotation_matrix);

            obj_outfile << setprecision(20) << time << " "
                                           << egm_T_wr(0,3)*SCALE << " "
                                           << egm_T_wr(1,3)*SCALE  << " "
                                           << egm_T_wr(2,3)*SCALE  << " "
                                           << new_quaternion.x() << " "<< new_quaternion.y() << " " << new_quaternion.z() << " " << new_quaternion.w() << "\n";


            //std::cout<<"out_put_count: "<<out_put_count<<std::endl;
            //std::cout<<"isometry3d: "<<std::endl<<Twr.matrix()<<std::endl;
            //std::cout<<"eigen 4d: "<<std::endl<<egm_T_wr<<std::endl;
        }
    }

    std::vector<std:: string> split(std:: string s, std:: string delimiter)
    {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std:: string token;
        std::vector<std:: string> res;

        while ((pos_end = s.find(delimiter, pos_start)) != std:: string::npos)
        {
            token = s.substr(pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back(token);
        }
        res.push_back(s.substr(pos_start));
        return res;
    }
    template<typename a>a string_to_num(const std:: string &str)
    {
        std::  istringstream iss(str);
        a num;
        iss >> num;
        return num;
    }

}
