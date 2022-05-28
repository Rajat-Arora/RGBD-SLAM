#pragma once

#include "common.h"

namespace rgbd_slam{

struct CAMERA_INTRINSIC_PARAMETERS;

class ParameterReader{

	public:
	ParameterReader(const string& filename = "./parameters.txt"){
		
		ifstream fin(filename.c_str());
		if(!fin){
			fin.open("../parameters.txt");
			if (!fin)
            {
                cerr<<"parameter file does not exist."<<endl;
                return;
            }
		}
	
		while(!fin.eof()){
			string str;
			getline(fin, str);
			if (str[0] == '#')  // Ignore line with a comment.
                continue;
		    int pos = str.find('#'); 
		    if(pos != -1){      
				str = str.substr(0, pos);  // Only keep the string with name and value ignore comment part.
			}
			pos = str.find("=");
			if (pos == -1)
                continue;  // If no equal to sign found continue.
			
			string key = str.substr( 0, pos ); 
            string value = str.substr( pos+1, str.length() );

			data[key] = value;  //Store it to std::map data

            if ( !fin.good() )
                break;
		}
	}

	public:
    map<string, string> data;

	template< class T >
    T getData( const string& key ) const
    {
        auto iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
        }
        return boost::lexical_cast<T>( iter->second ); //used to convert string to T type variable.
    }
	
	rgbd_slam::CAMERA_INTRINSIC_PARAMETERS getCamera() const; //will be defined later


};

}
