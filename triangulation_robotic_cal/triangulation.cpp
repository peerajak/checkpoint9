#include <iostream>
#include <cmath>
#include <tuple>
std::tuple<double, double> get_pmid_o(double p1x, double p1y, double p2x, double p2y,
double a, double b, double c, double alpha2, double theta1, double theta2, 
double laserx, double lasery){
   double R11 = std::cos(alpha2);
   double R12 = -1*std::sin(alpha2);
   double R21 = std::sin(alpha2);
   double R22 = std::cos(alpha2);

   double P1x_laser_coordinate = b*std::cos(theta1);
   double P1y_laser_coordinate = b*std::sin(theta1);
   double P2x_laser_coordinate = c*std::cos(theta2);
   double P2y_laser_coordinate = c*std::sin(theta2);

   double P1x_origin_coordinate =  R11*P1x_laser_coordinate +R12*P1y_laser_coordinate + laserx;
   double P1y_origin_coordinate =  R21*P1x_laser_coordinate +R22*P1y_laser_coordinate + lasery;
   double P2x_origin_coordinate =  R11*P2x_laser_coordinate +R12*P2y_laser_coordinate + laserx;
   double P2y_origin_coordinate =  R21*P2x_laser_coordinate +R22*P2y_laser_coordinate + lasery;

   double Pmidx_origin_coordinate = (P1x_origin_coordinate + P2x_origin_coordinate)/2;
   double Pmidy_origin_coordinate = (P1y_origin_coordinate + P2y_origin_coordinate)/2;

    return std::tuple<double,double>{Pmidx_origin_coordinate,Pmidy_origin_coordinate};
}

std::tuple<double, double> get_pmid_laser(double p1x, double p1y, double p2x, double p2y,
double a, double b, double c, double alpha2, double theta1, double theta2, 
double laserx, double lasery){
   double R11 = std::cos(alpha2);
   double R12 = -1*std::sin(alpha2);
   double R21 = std::sin(alpha2);
   double R22 = std::cos(alpha2);

   double P1x_laser_coordinate = b*std::cos(theta1);
   double P1y_laser_coordinate = b*std::sin(theta1);
   double P2x_laser_coordinate = c*std::cos(theta2);
   double P2y_laser_coordinate = c*std::sin(theta2);

   double Pmidx_laser_coordinate = (P1x_laser_coordinate + P2x_laser_coordinate)/2;
   double Pmidy_laser_coordinate = (P1y_laser_coordinate + P2y_laser_coordinate)/2;

    return std::tuple<double,double>{Pmidx_laser_coordinate,Pmidy_laser_coordinate};
}

std::tuple<double, double> get_p1_to_p2_perpendicular_vector_laser_coordinate(double p1x_laser, double p1y_laser, double p2x_laser, double p2y_laser){
    double p1_to_p2_laser_x = p2x_laser - p1x_laser;
    double p1_to_p2_laser_y = p2y_laser - p1y_laser;
    double perpen_l1_to_l2_laser_x = 1;
    double perpen_l1_to_l2_laser_y = -p1_to_p2_laser_x/p1_to_p2_laser_y;
     return std::tuple<double,double>{perpen_l1_to_l2_laser_x,perpen_l1_to_l2_laser_y};
}

double yaw_degree_radian_between_perpendicular_and_Pmid(double p1p2_perpendicular_x_laser, double p1p2_perpendicular_y_laser, double pmid_x_laser,double pmid_y_laser){
   double pmid_dot_p1p2_perpendicular_laser = p1p2_perpendicular_x_laser* pmid_x_laser + p1p2_perpendicular_y_laser*pmid_y_laser;
   double size_of_p1p2_perpendicular_laser = std::sqrt( p1p2_perpendicular_x_laser*p1p2_perpendicular_x_laser+ p1p2_perpendicular_y_laser*p1p2_perpendicular_y_laser);
   double size_of_pmid_x_laser = std::sqrt(pmid_x_laser*pmid_x_laser+pmid_y_laser*pmid_y_laser);
   return std::acos(pmid_dot_p1p2_perpendicular_laser/(size_of_p1p2_perpendicular_laser*size_of_pmid_x_laser));
}

double yaw_degree_radian_between_perpendicular_and_laser_x(double p1p2_perpendicular_x_laser, double p1p2_perpendicular_y_laser){
   double x_laser = 1.0;
   double y_laser = 0;
   double x_laser_dot_p1p2_perpendicular_laser = p1p2_perpendicular_x_laser* x_laser + p1p2_perpendicular_y_laser*y_laser;
   double size_of_p1p2_perpendicular_laser = std::sqrt( p1p2_perpendicular_x_laser*p1p2_perpendicular_x_laser+ p1p2_perpendicular_y_laser*p1p2_perpendicular_y_laser);
   double size_of_x_laser = 1;
   return std::acos(x_laser_dot_p1p2_perpendicular_laser/(size_of_p1p2_perpendicular_laser*size_of_x_laser));
}


//std::tuple<double, double>
int main(void){
   std::cout<<"Calculating linear algebra"<<std::endl;
   double p1x = 6.0, p1y = 8.0;
   double p2x = 10.0, p2y = -2.0;
   double a=5.0, b= 7.21246706257271, c= 5.830281871243811;
   double alpha2 = 0.17453292519943295;
   double theta1 = 1.24652373718018,theta2 = -0.6888675951149327;
   double laserx = 4.924039;
   double lasery = 0.8682409;

   double R11 = std::cos(alpha2);
   double R12 = -1*std::sin(alpha2);
   double R21 = std::sin(alpha2);
   double R22 = std::cos(alpha2);

   double P1x_laser_coordinate = b*std::cos(theta1);
   double P1y_laser_coordinate = b*std::sin(theta1);
   double P2x_laser_coordinate = c*std::cos(theta2);
   double P2y_laser_coordinate = c*std::sin(theta2);

   double P1x_origin_coordinate =  R11*P1x_laser_coordinate +R12*P1y_laser_coordinate + laserx;
   double P1y_origin_coordinate =  R21*P1x_laser_coordinate +R22*P1y_laser_coordinate + lasery;
   double P2x_origin_coordinate =  R11*P2x_laser_coordinate +R12*P2y_laser_coordinate + laserx;
   double P2y_origin_coordinate =  R21*P2x_laser_coordinate +R22*P2y_laser_coordinate + lasery;

   std::cout<<"P1 = ["<<P1x_origin_coordinate<<","<<P1y_origin_coordinate<<"]"<<std::endl;
   std::cout<<"P2 = ["<<P2x_origin_coordinate<<","<<P2y_origin_coordinate<<"]"<<std::endl;

   double Pmidx_origin_coordinate = (P1x_origin_coordinate + P2x_origin_coordinate)/2;
   double Pmidy_origin_coordinate = (P1y_origin_coordinate + P2y_origin_coordinate)/2;
   std::cout<<"Pmid = ["<<Pmidx_origin_coordinate<<","<<Pmidy_origin_coordinate<<"]"<<std::endl;

   std::cout<<"-------using function----"<<std::endl;
  
   std::tuple<double, double> result_pmid = get_pmid_o(p1x,p1y,p2x,p2y,a,b,c,alpha2,theta1,theta2,laserx,lasery);
   double Pmidx_origin = std::get<0>(result_pmid);
   double Pmidy_origin = std::get<1>(result_pmid); 
   std::cout<<"Pmid = ["<<Pmidx_origin<<","<<Pmidy_origin<<"]"<<std::endl;

   std::tuple<double, double> result_pmid_laser = get_pmid_laser(p1x,p1y,p2x,p2y,a,b,c,alpha2,theta1,theta2,laserx,lasery);
   double Pmidx_laser = std::get<0>(result_pmid_laser);
   double Pmidy_laser = std::get<1>(result_pmid_laser); 
   std::cout<<"Pmid_laser = ["<<Pmidx_laser<<","<<Pmidy_laser<<"]"<<std::endl;

   std::tuple<double, double> result_p1p2_perpendicular_laser = get_p1_to_p2_perpendicular_vector_laser_coordinate
       (P1x_laser_coordinate,P1y_laser_coordinate,P2x_laser_coordinate,P2y_laser_coordinate);
   double p1p2_perpendicular_x_laser = std::get<0>(result_p1p2_perpendicular_laser);
   double p1p2_perpendicular_y_laser = std::get<1>(result_p1p2_perpendicular_laser); 
   std::cout<<"p1p2_perpendicular 1st answer = ["<<p1p2_perpendicular_x_laser<<","<<p1p2_perpendicular_y_laser<<"]"<<std::endl;
   std::cout<<"p1p2_perpendicular 2nd answer = ["<<-p1p2_perpendicular_x_laser<<","<<-p1p2_perpendicular_y_laser<<"]"<<std::endl;

   double yaw_degree_perpendicular_Pmid = yaw_degree_radian_between_perpendicular_and_Pmid(p1p2_perpendicular_x_laser,p1p2_perpendicular_y_laser,Pmidx_laser, Pmidy_laser);
    std::cout<<"yaw_degree_perpendicular_Pmid = "<<yaw_degree_perpendicular_Pmid <<std::endl;

  double yaw_degree_radian_perpendicular_and_laser_x = yaw_degree_radian_between_perpendicular_and_laser_x(p1p2_perpendicular_x_laser,p1p2_perpendicular_y_laser);
      std::cout<<"yaw_degree_radian_perpendicular_and_laser_x = "<<yaw_degree_radian_perpendicular_and_laser_x <<std::endl;
  return 0;
}
