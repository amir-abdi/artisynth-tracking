#ifndef CONFIG_H
#define CONFIG_H


#include <string>
#include <map>
#include <rapidxml.hpp>
#include <streambuf>
#include <fstream>
#include <iostream>
using namespace std;
using namespace rapidxml;

class Config
{
public:
	static map<string, string> config;
	
	static bool digitize_flag;
	static bool debug_output_flag;
	static int sleep_time_capture;
	static int FORBody_index;
	static string server_name;
	static string server_name_raw;
	
	static bool write_tranformations_flag;
	static bool write_raw_flag;
	static string write_format;
	
	static int upper_index;
	static int lower_index;
	static int stylus_index;

	static map<string, string> get_config();
	static void InitConfig();
	static void ParseConfig(map<string, string> config);
	static map<string, string> read_config_file(string file);

	static bool register_on_the_fly;
};




#endif // ! CONFIG_H