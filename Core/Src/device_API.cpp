/*
 * device_API.cpp
 *
 *  Created on: 3 июл. 2023 г.
 *      Author: Ierixon-HP
 */

#include "flash_spi.h"
#include "LED.h"
#include "lwip.h"
using namespace std;
#include <string>
#include "api.h"
#include <iostream>
#include <vector>
#include <externDriver.hpp>
#include "device_API.h"


/*variables ---------------------------------------------------------*/
extern settings_t settings;
extern flash mem_spi;
extern extern_driver *pMotor;
extern struct netif gnetif;

/* Typedef -----------------------------------------------------------*/
struct mesage_t{
	uint32_t cmd;
	uint32_t addres_var;
	uint32_t data_in;
	uint32_t data_in1;
	bool need_resp = false;
	bool data_in_is;
	uint32_t data_out;
	string err; // сообщение клиенту об ошибке в сообщении
	bool f_bool = false; // наличие ошибки в сообшении
};

//Флаги для разбора сообщения
string f_cmd("C");
string f_addr("A");
string f_datd("D");
string f_datd1("N");
string delim("x");

string Command_execution(string in_str){
	// Парсинг
	vector<string> arr_msg;
	vector<mesage_t> arr_cmd;
	size_t prev = 0;
	size_t next;
	size_t delta = delim.length();
	bool errMSG = false;

	//разбить на сообщения
	while( ( next = in_str.find( delim, prev ) ) != string::npos ){
		arr_msg.push_back( in_str.substr( prev, (next +1)-prev ) );
		prev = next + delta;
	}
	//arr_msg.push_back( in_str.substr( prev ) );
	if(arr_msg.size() != 0){
		//занести сообщения в структуру
		int count_msg = arr_msg.size();
		for (int i = 0; i < count_msg; ++i) {
			prev = 0;
			next = 0;
			size_t posC = 0;
			size_t posA = 0;
			size_t posD = 0;
			size_t posN = 0;
			size_t posx = 0;
			mesage_t temp_msg;

			// выделение комманды
			delta = f_cmd.length();
			next = arr_msg[i].find(f_cmd);
			posC = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in C flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;

			}
			prev = next + delta;

			// выделение адреса
			delta = f_addr.length();
			next = arr_msg[i].find(f_addr, prev);
			posA = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in A flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных
			delta = f_datd.length();
			next = arr_msg[i].find(f_datd, prev);
			posD = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in D flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных 1
			delta = f_datd1.length();
			next = arr_msg[i].find(f_datd1, prev);
			posN = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in N flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}
			prev = next + delta;

			// выделение данных
			delta = delim.length();
			next = arr_msg[i].find(delim, prev);
			posx = next;
			if(next == string::npos){
				//Ошибка
				temp_msg.err = "wrong format in x flag";
				errMSG = true;
				arr_cmd.push_back(temp_msg);
				continue;
			}

			bool isNum;
			// from flag "C" to flag "A" is a number ?
			isNum = isNumeric(arr_msg[i].substr(posC +1, (posA -1) - posC));

			// chain, check parameters
			if(isNum){
				temp_msg.cmd = (uint32_t)stoi(arr_msg[i].substr(posC +1, (posA -1) - posC)); // get number from string
				// from flag "A" to flag "D" is a number ?
				isNum = isNumeric(arr_msg[i].substr(posA +1, (posD -1) - posA));

				if(isNum){
					temp_msg.addres_var = (uint32_t)stoi(arr_msg[i].substr(posA +1, (posD -1) - posA)); // get number from string
					// from flag "D" to flag "N" is a number ?
					isNum = isNumeric(arr_msg[i].substr(posD +1, (posN -1) - posD));

					if(isNum){
						temp_msg.data_in = (uint32_t)stoi(arr_msg[i].substr(posD +1, (posN -1) - posD)); // get number from string
						// from flag "N" to flag "x" is a number ?
						isNum = isNumeric(arr_msg[i].substr(posN +1, (posx -1) - posN));

						if(isNum){
							temp_msg.data_in1 = (uint32_t)stoi(arr_msg[i].substr(posN +1, (posx -1) - posN)); // get number from string
						}
						else{
							temp_msg.err = "err after N is not number";
							errMSG = true;
						}
					}
					else{
						temp_msg.err = "err after D is not number";
						errMSG = true;
					}
				}
				else{
					temp_msg.err = "err after A is not number";
					errMSG = true;
				}
			}
			else{
				temp_msg.err = "err after C is not number";
				errMSG = true;
			}

			arr_cmd.push_back(temp_msg);
		}
	}
	else{
		mesage_t temp_msg;
		temp_msg.err = "err format message";
		arr_cmd.push_back(temp_msg);
		errMSG = true;
	}
	// Закончили парсинг

	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//Выполнение комманд
	int count_cmd = arr_cmd.size();
	if(!errMSG){
		for (int i = 0; i < count_cmd; ++i) {
			switch (arr_cmd[i].cmd) {
			case 1: // start/stop
				// добавить статус
				if(arr_cmd[i].addres_var == 1)
				{
					arr_cmd[i].data_out = (uint32_t)pMotor->getStatusRotation();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";

				} else {
                    // Проверка состояния драйвера перед выполнением команды
                    if (pMotor->getDriverStatus() == DRIVER_ERROR) {
                        arr_cmd[i].err = " Driver Error ";
                        arr_cmd[i].f_bool = true;
                        break;
                    }

					switch (arr_cmd[i].data_in) {
						case 0:
						{
							pMotor->removeBreak(false);
							pMotor->stop(statusTarget_t::finished);
							//pMotor->slowdown();
							arr_cmd[i].err = " OK ";
							break;
						}
						case 1:
						{

							pMotor->removeBreak(true);
							if (pMotor->gotoInfinity())
								arr_cmd[i].err = " OK ";
							else
								arr_cmd[i].err = " noStart ";
							break;
						}
						/*
						case 2:
						{
							if(pMotor->gotoLSwitch(0)){
								arr_cmd[i].err = " OK ";
							} else {
								arr_cmd[i].err = " not calibrated ";
								arr_cmd[i].f_bool = true;
							}
							break;
						}*/
						default:
						{
							arr_cmd[i].err = " ERR ";
							break;
						}
					}
				}

				break;
			case 2: // Call
				pMotor->CallStart();
				arr_cmd[i].err = " Start call ";
				break;
			case 3: // Speed
				if(arr_cmd[i].addres_var == 0){ // set
					pMotor->SetSpeed(arr_cmd[i].data_in);
					pMotor->SetStartSpeed(arr_cmd[i].data_in1);
					arr_cmd[i].err = " OK ";
				} else {				//get
					arr_cmd[i].data_out = (uint32_t)pMotor->getSpeed();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
				}
				break;
			case 5: // statusTarget
				switch (arr_cmd[i].addres_var) {
				case 0: //Target
					arr_cmd[i].data_out = (uint32_t)pMotor->getStatusTarget();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
					break;
				case 1: // на концевиках или нет
					arr_cmd[i].data_out = (uint32_t)pMotor->get_pos();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
					break;
				case 2: // количество шагов (позиция относительно 0 концевика)
					arr_cmd[i].data_out = pMotor->getCurrentSteps();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
					break;
				case 3: // состояние драйвера
				    arr_cmd[i].data_out = (uint32_t)pMotor->getDriverStatus();
				    arr_cmd[i].need_resp = true;
				    if (!pMotor->isDriverStatusValid()) {
				        arr_cmd[i].err = " Status not valid ";
				    } else {
				        arr_cmd[i].err = " OK ";
				    }
				    break;
				default:
					arr_cmd[i].err = "A... wrong parameter";
					arr_cmd[i].f_bool = true;
					break;
				}
				break;
			case 6: // Acceleration-Slowdown
				switch (arr_cmd[i].addres_var) {
				case 0:
					pMotor->SetAcceleration(arr_cmd[i].data_in);
					mem_spi.Write(settings);
					arr_cmd[i].err = " OK ";
					break;
				case 1:
					arr_cmd[i].data_out = (uint32_t)pMotor->getAcceleration();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
					break;
				case 2:
					pMotor->SetSlowdown(arr_cmd[i].data_in);
					arr_cmd[i].err = " OK ";
					break;
				case 3:
					arr_cmd[i].data_out = (uint32_t)pMotor->getSlowdown();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
					break;
				default:
					arr_cmd[i].err = "A... wrong parameter";
					arr_cmd[i].f_bool = true;
					break;
				}
				break;
			case 7: // steps for started slowdown SetSlowdownDistance

				break;
			case 8: // Direct
				if(arr_cmd[i].addres_var == 0){
					if((!arr_cmd[i].data_in) && pMotor->getStatusRotation() == statusMotor :: STOPPED)
						pMotor->SetDirection(dir::CW);
					else if((arr_cmd[i].data_in) && pMotor->getStatusRotation() == statusMotor :: STOPPED)
						pMotor->SetDirection(dir::CCW);
					else{
						arr_cmd[i].err = "motor not stopped";
						arr_cmd[i].f_bool = true;
					}
					arr_cmd[i].err = " OK ";
				} else {
					arr_cmd[i].data_out = (uint32_t)pMotor->getStatusDirect();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
				}
				break;
			case 9://Mode rotation
				switch (arr_cmd[i].addres_var) {
					case 0:
						pMotor->SetMode((mode_rotation_t)arr_cmd[i].data_in);
						arr_cmd[i].err = " OK ";
						break;
					case 1:
						arr_cmd[i].data_out = (uint32_t)pMotor->getMode();
						arr_cmd[i].need_resp = true;
						arr_cmd[i].err = " OK ";
						break;
					default:
						arr_cmd[i].err = " err ";
						break;
				}
				break;
			case 10: // TimeOut
				if(arr_cmd[i].addres_var == 0){
					pMotor->setTimeOut(arr_cmd[i].data_in);
					arr_cmd[i].err = " OK ";
				} else {
					arr_cmd[i].data_out = (uint32_t)pMotor->getTimeOut();
					arr_cmd[i].need_resp = true;
					arr_cmd[i].err = " OK ";
				}
				break;
			case 11: // save
				mem_spi.W25qxx_EraseSector(0);
				osDelay(5);
				mem_spi.Write(settings);
				arr_cmd[i].err = "OK";
				break;
			case 12: // Reboot
				if(arr_cmd[i].data_in){
					NVIC_SystemReset();
				}
				arr_cmd[i].err = "OK";
				break;
			case 13: // DHCP
				settings.DHCPset = (uint8_t)arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 14: // IP
				settings.saveIP.ip[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 15: // MASK
				settings.saveIP.mask[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 16: // GW
				settings.saveIP.gateway[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case 17: // MAC
				settings.MAC[arr_cmd[i].addres_var] = arr_cmd[i].data_in;
				arr_cmd[i].err = "OK";
				break;
			case CMD_SAVE_POINT:
			    if(arr_cmd[i].addres_var == 0) {
			        // Сохранить текущую позицию как точку
			        if(pMotor->saveCurrentPositionAsPoint(arr_cmd[i].data_in)) {
			            arr_cmd[i].err = " OK ";
			        } else {
			            arr_cmd[i].err = " Error saving point ";
			            arr_cmd[i].f_bool = true;
			        }
			    }
			    break;

			case CMD_GET_POSITION:
			    if(arr_cmd[i].addres_var == 1) {
			        // Получить текущую позицию
			        arr_cmd[i].data_out = pMotor->getCurrentSteps();
			        arr_cmd[i].need_resp = true;
			        arr_cmd[i].err = " OK ";
			    } else if(arr_cmd[i].addres_var == 2) {
			        // Получить номер текущей точки
			        arr_cmd[i].data_out = pMotor->getCurrentPoint();
			        arr_cmd[i].need_resp = true;
			        arr_cmd[i].err = " OK ";
			    } else if(arr_cmd[i].addres_var == 3) {
			        // Получить статус калибровки
			        arr_cmd[i].data_out = pMotor->isCalibrated() ? 1 : 0;
			        arr_cmd[i].need_resp = true;
			        arr_cmd[i].err = " OK ";
			    }
			    break;

			case CMD_GOTO_SW0:
					if (pMotor->gotoLSwitch(0)) {
						arr_cmd[i].err = " OK ";
					} else {
			            arr_cmd[i].err = " Error moving to sw0 ";
			            arr_cmd[i].f_bool = true;
					}
			    break;

			case CMD_GOTO_SW1:
					if (pMotor->gotoLSwitch(1)) {
						arr_cmd[i].err = " OK ";
					} else {
						arr_cmd[i].err = " Error moving to sw1 ";
						arr_cmd[i].f_bool = true;
					}
			    break;

			case CMD_GOTO_POINT:
			    if(arr_cmd[i].addres_var == 0) {
			        if(pMotor->gotoPoint(arr_cmd[i].data_in)) {
			            arr_cmd[i].err = " OK ";
			        } else {
			            arr_cmd[i].err = " Error moving to point ";
			            arr_cmd[i].f_bool = true;
			        }
			    }
			    break;

			case CMD_GET_POINT:
			    if(arr_cmd[i].addres_var == 1) {
			    	if(arr_cmd[i].data_in < 10)
			    	{
						arr_cmd[i].data_out = settings.points.points[arr_cmd[i].data_in];
						arr_cmd[i].need_resp = true;
						arr_cmd[i].err = " OK ";
			    	}else {
			            arr_cmd[i].err = " Error number point ";
			            arr_cmd[i].f_bool = true;
			    	}

			    }
			    break;

			case CMD_GOTO_POSITION:
			    if(arr_cmd[i].addres_var == 0) {
			        if(pMotor->gotoPosition(arr_cmd[i].data_in)) {
			            arr_cmd[i].err = " OK ";
			        } else {
			            arr_cmd[i].err = " Error moving to position ";
			            arr_cmd[i].f_bool = true;
			        }
			    }
			    break;

			case CMD_GET_MAX_POSITION:
			    if(arr_cmd[i].addres_var == 1) {
			        arr_cmd[i].data_out = pMotor->getMaxPosition();
			        arr_cmd[i].need_resp = true;
			        arr_cmd[i].err = " OK ";
			    }
			    break;

			case CMD_SET_POINT:
			    if(arr_cmd[i].addres_var == 0) {
			        pMotor->setPoint(arr_cmd[i].data_in1, arr_cmd[i].data_in);
			        arr_cmd[i].err = " OK ";
			    }
			    break;
			default:
				arr_cmd[i].err = "Command does not exist";
				arr_cmd[i].f_bool = true;
				break;
			}
		}
	}
	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//Формируем ответ
	string resp;
	if(!errMSG){
		for (int i = 0; i < count_cmd; ++i) {

			if(arr_cmd[i].f_bool == false){
				resp.append(f_cmd + to_string(arr_cmd[i].cmd));
				if(arr_cmd[i].need_resp){
					resp.append(f_datd + to_string(arr_cmd[i].data_out));
				}else{
					resp.append(" " + arr_cmd[i].err);
				}
				resp.append(delim);
			}
			else{
				resp.append(arr_cmd[i].err);
			}

		}
	}
	else{
		resp.append(arr_cmd[0].err);
	}

	return resp;
}


bool isNumeric(std::string const &str)
{
	char* p;
	strtol(str.c_str(), &p, 10);
	return *p == 0;
}
