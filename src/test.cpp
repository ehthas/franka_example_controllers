// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

//#include <fstream>
//#include <iterator>
//#include <cmath>
//#include <iostream>
//#include <array>
//#include <vector>
//#include <boost/numeric/odeint.hpp>

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>


using namespace std; 

//#define PORT 8080

//using namespace boost::numeric::odeint;


//typedef std::vector<double> state_type;


//void my_system(const state_type &s, state_type &dsdt, double time)
//{
   
   //dsdt[0] =  -2*s[0];
   //dsdt[1] =  -2*s[1];
   //dsdt[2] =  -2*s[2];
   //dsdt[3] =  -2*s[3];
   //dsdt[4] =  -2*s[4];
   //dsdt[5] =  -2*s[5];
   //dsdt[6] =  -2*s[6]; 

//}

// observer function to display output after integration
//void my_observer( const state_type &s , const double time) 
//{  
  // std::cout << time << '\t' << "s =  " 
  //                   << s[0] << "     " 
  //                   << s[1] << "     " 
  //                   << s[2] << "     " 
  //                   << s[3] << "     " 
  //                   << s[4] << "     " 
  //                   << s[5] << "     " 
  //                   << s[6] <<  std::endl;
//}   


int main(int argc, char** argv) {

    //UDP connection setup
    
    struct sockaddr_in cpp_addr, matlab_addr, sender_addr;

    // C++ UDP Port

    cpp_addr.sin_family = AF_INET;
    cpp_addr.sin_addr.s_addr = inet_addr("172.16.0.1");   //10.11.104.24
    cpp_addr.sin_port = htons(16385);


    // Matlab UDP Port

    matlab_addr.sin_family = AF_INET;
    matlab_addr.sin_addr.s_addr = inet_addr("172.16.0.3");
    matlab_addr.sin_port = htons(8080);

    // Create and Bind socket to local address

    int client = socket(AF_INET, SOCK_DGRAM, 0);

    if (client < 0)
    {
       std::cout << "Error creating socket" << std::endl;
       exit(1);
    }

    std::cout << "Socket created" << std::endl;

    if ((bind(client, (struct sockaddr*)&cpp_addr, sizeof(cpp_addr)) ) < 0)
    {
       std::cout << "Error binding connection" << std::endl;
       exit(1);

    }

    int recvlen;
    int bufSize = 2048;
    unsigned char buf[bufSize];
    socklen_t addrlen = sizeof(matlab_addr);

    //double w = 0.0;

    //char const * message;
    
    for(;;){

    std::cout << "Waiting to receive" << std::endl;

    recvlen = recvfrom(client, buf, bufSize, 0, (struct sockaddr *)&sender_addr, &addrlen);
    //recvlen = recvfrom(client, buf, bufSize, 0, &addrlen);

    if (recvlen > 0) {
       buf[recvlen] = 0;
    }else{
       std::cout << "Nothing received" << std::endl;
       exit(1);
    } 


   double (* bufDouble) = reinterpret_cast<double *>(buf);

/*   std::cout << "Received is" << bufDouble[0] << std::endl;
   std::cout << "Received is" << bufDouble[1] << std::endl;
   std::cout << "Received is" << bufDouble[2] << std::endl;
   std::cout << "Received is" << bufDouble[3] << std::endl;
   std::cout << "Received is" << bufDouble[4] << std::endl;
   std::cout << "Received is" << bufDouble[5] << std::endl;
*/
   std::cout << "Received is" << bufDouble[0] << " "
                              << bufDouble[1] << " "
                              << bufDouble[2] << " "
                              << bufDouble[3] << " "
                              << bufDouble[4] << " "
                              << bufDouble[5] << std::endl;



   //printf("received %f", bufDouble[0]);

   }







//  if (argc != 2) {
//    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//    return -1;
//  }
/*  state_type s(7), s_prev(7);
  state_type disp_vec;
  s[0] = sqrt(10*10);
  s[1] = sqrt(10*10);
  s[2] = sqrt(10*10);
  s[3] = sqrt(10*10);
  s[4] = sqrt(10*10);
  s[5] = sqrt(10*10);
  s[6] = sqrt(10*10);  

  s_prev[0] = sqrt(10*10);
  s_prev[1] = sqrt(10*10);
  s_prev[2] = sqrt(10*10);
  s_prev[3] = sqrt(10*10);
  s_prev[4] = sqrt(10*10);
  s_prev[5] = sqrt(10*10);
  s_prev[6] = sqrt(10*10); 

  double t = 0.0;
  double dt = 0.001; 
  double t1=t+dt;
*/
  //ofstream out_file("outfile.txt");
  //ostream_iterator<double> out_iterator(out_file,"\n");
  //copy(s.begin(),s.end(),out_iterator);       
  //fstream outfile;
  //outfile.open("outfile.txt",ios::out);  // write to file  
  //if (outfile.is_open())
  //{ 
     //for (size_t i=0; i<t1+dt; i+dt)
     //{   
  //   outfile << t << '\t' << "s =  " 
  //                        << s[0] << "     " 
  //                        << s[1] << "     " 
  //                        << s[2] << "     " 
  //                        << s[3] << "     " 
  //                        << s[4] << "     " 
  //                        << s[5] << "     " 
  //                        << s[6] <<  std::endl;

      //integrate(my_system, s, t , t1, dt, my_observer);
  //    outfile.close();
      //t +=dt;
  //}
/*
  s[0] = s_prev[0] - 2*s_prev[0];
  s[1] = s_prev[1] - 2*s_prev[1];
  s[2] = s_prev[2] - 2*s_prev[2];
  s[3] = s_prev[3] - 2*s_prev[3];
  s[4] = s_prev[4] - 2*s_prev[4];
  s[5] = s_prev[5] - 2*s_prev[5];
  s[6] = s_prev[6] - 2*s_prev[6];

  s_prev[0] = s[0];
  s_prev[1] = s[1];
  s_prev[2] = s[2];
  s_prev[3] = s[3];
  s_prev[4] = s[4];
  s_prev[5] = s[5];
  s_prev[6] = s[6];
*/
   
}
