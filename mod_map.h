//
// Created by lzw on 2021/11/18.
//

#ifndef FP_MAP_MOD_MAP_H
#define FP_MAP_MOD_MAP_H

#include "../include/utility.hpp"
#include "../include/ANNF.hpp"
#include "../include/CadMap.hpp"
#include "../include/Shape.hpp"


//#include "../include/CeresCostFunctor.hpp"


namespace  lzw_mod
{
    class abc_2d_line
    {   public:
        pcl::PointXYZ start_;
        pcl::PointXYZ end_;
        double a0_;
        double b0_;
        double c0_;

        /*friend ostream & operator<<( ostream & os,const abc_2d_line & c);

        ostream & operator<<( ostream & os,const abc_2d_line & c)
        {
            os <<c.start_<<" "<<c.end_<<endl; //以"a+bi"的形式输出
            return os;
        }*/
        abc_2d_line(pcl::PointXYZ start,pcl::PointXYZ end,double a,double b,double c)
        {
            start_=start;
            end_=end;
            a0_=a;
            b0_=b;
            c0_=c;
        }

        abc_2d_line(pcl::PointXYZ start,pcl::PointXYZ end){
            start_=start;
            end_=end;

            a0_=start_.y-end_.y;
            b0_=end_.x-start_.x;
            c0_=start_.x*end_.y-start_.y*end_.x;
        }
        abc_2d_line() {};
    };
    class mod_map
    {
        public:
        pcl::PointCloud<PointType>::Ptr cad_cloud_;
        CadMap vis_map_;
        CadMap opt_map_;
        ANNF vis_annf_;
        ANNF annf_;

        mod_map();
        //void draw_multi_trajectory(std::vector<Line *> v_shapes);
         void draw_multi_trajectory();

        //draing points, lines,pose ,and map itself
        //whether draw map itself is control by the draw_itself_flag
        //v_draw_flags contrals whether draw the corrsponding matter
        //v[0] <==> v_l
        //v[1] <==> poses
        //v[2] <==> v_p_pcds
        //v[3] <==> map_itself
        //v[4] <==> whether linking the poses with line
          void  draw_multi_trajectory(std::vector<abc_2d_line> &v_l,
                                             std::vector<Eigen::Matrix4d,
                                                     Eigen::aligned_allocator<Eigen::Matrix4d>> poses,
                                             std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > v_p_pcds,
                                             double axis_size,
                                      vector<double> v_draw_flags);

          //calculate 2d intersection
          //to do add 3d intersection ,maybe overloading function
        static pcl::PointXYZ calculate_intersection(pcl::PointXYZ &a,pcl::PointXYZ&b,
                                             pcl::PointXYZ &c,pcl::PointXYZ &d);
        static int is_intersection(pcl::PointXYZ a,pcl::PointXYZ b,
                            pcl::PointXYZ c,pcl::PointXYZ d);

        //input: v_abs_poses  given a series pose, render 2d depths
        //output: v_p_pcds

        void render_2d_depth(std::vector<Eigen::Matrix4d,
                Eigen::aligned_allocator<Eigen::Matrix4d>> v_abs_poses,
                             std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & v_p_pcds,
                             Eigen::Matrix3d egm_cam_K);
    };

    void read_txt(std:: string file,
                  std::vector<Eigen::Matrix4d,
                  Eigen::aligned_allocator<Eigen::Matrix4d>> &v_abs_pose);
    void read_txt_no_mod(std:: string file,
                         std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_abs_pose);


    std::vector<std:: string> split(std:: string s, std:: string delimiter);
    template<typename a>a string_to_num(const std:: string &str);
}
#endif //FP_MAP_MOD_MAP_H
