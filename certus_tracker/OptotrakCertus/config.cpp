#include "config.h"

map<string, string> Config::config;
bool Config::digitize_flag;
bool Config::debug_output_flag;
int Config::sleep_time_capture;
int Config::FORBody_index;
string Config::server_name;
string Config::server_name_raw;
bool Config::write_tranformations_flag;
bool Config::write_raw_flag;
string Config::write_format;
int Config::upper_index;
int Config::lower_index;
int Config::stylus_index;
bool Config::register_on_the_fly;

void Config::ParseConfig(map<string, string> config)
{
	Config::digitize_flag = strcmp(config["digitize"].c_str(), "true") == 0 ? true : false;
	Config::debug_output_flag = strcmp(config["debug"].c_str(), "true") == 0 ? true : false;	
	
	Config::write_raw_flag = strcmp(config["write_raw_data"].c_str(), "true") == 0 ? true : false;
	Config::write_tranformations_flag = strcmp(config["write_raw_data"].c_str(), "true") == 0 ? true : false;

	Config::sleep_time_capture = atoi(config["sleep"].c_str());
	Config::FORBody_index = atoi(config["FORBody"].c_str());
	Config::server_name = config["server_name"];	
	Config::server_name_raw = config["server_name_raw"];

	Config::upper_index = atoi(config["upper_index"].c_str());
	Config::lower_index = atoi(config["lower_index"].c_str());
	Config::stylus_index = atoi(config["stylus_index"].c_str());

	Config::write_format = config["write_format"];

	Config::register_on_the_fly = strcmp(config["register_on_the_fly"].c_str(), "true") == 0 ? true : false;
}

map<string, string> Config::read_config_file(string file)
{

	std::ifstream t(file);
	std::string str((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());
	xml_document<char> doc;
	doc.parse<0>((char*)str.c_str());

	map<string, string> config;
	xml_node<> *first_node = doc.first_node()->first_node()->first_node();

	for (xml_node<> *node = first_node; node; node = node->next_sibling())
	{
		config[node->first_attribute("key")->value()] = node->first_attribute("value")->value();
		//cout << node->first_attribute("key")->value() << endl << node->first_attribute("value")->value();
	}
	return config;
}

void Config::InitConfig()
{
	Config::config = read_config_file("config.config");		
	ParseConfig(Config::config);
}

map<string, string> Config::get_config()
{
	return config;
}