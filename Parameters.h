#pragma once
#include <iostream>
const int sigma_ratio = 2;

namespace idi {
	struct CDCParameters {
		int deltaCDC = 4;
		double rhoColor = 0.15;
		double rhoDisp = 20;
		//double rhoCost = 0.01;
		double rhoCost = 1;
		double tauColor = 3;
		double tauDisp = 1;
		double epsDisp = 0.5;
		//double epsDisp = 1;

		static std::vector<std::string> parameterList() {
			std::vector<std::string> parameters;
			parameters.push_back("rhoColor");
			parameters.push_back("rhoDisp");
			parameters.push_back("rhoCost");
			parameters.push_back("tauColor");
			parameters.push_back("tauDisp");
			parameters.push_back("epsDisp");
			parameters.push_back("deltaCDC");
			return parameters;
		}
	};

	struct Parameters {
		CDCParameters cdcParams = CDCParameters();
		double t0 = 10;
		double alpha = 0.8;
		double sigma = 0.04;
		double lambda0 = 100;
		double gamma0 = 0.16;
		int qCdc = 2;
		int qSG = 6;
		int nIter = 2;
		int nTemps = 5;
		double tau_k = 1.0;
		double tau_d = 0.031;
		int deltaA = 5;
		double innerScale = 1.4;
		double outerScale = 1.4;
		int deltaAvg = 18;

		void changeValueFromString(std::string string, double value) {
			if (!string.compare("deltaCDC")) {
				cdcParams.deltaCDC = value;
			}
			else {
				if (!string.compare("rhoColor")) {
					cdcParams.rhoColor = value;
				}
				else {
					if (!string.compare("rhoDisp")) {
						cdcParams.rhoDisp = value;
					}
					else {
						if (!string.compare("rhoCost")) {
							cdcParams.rhoCost = value;
						}
						else {
							if (!string.compare("tauColor")) {
								cdcParams.tauColor = value;
							}
							else {
								if (!string.compare("tauDisp")) {
									cdcParams.tauDisp = value;
								}
								else {
									if (!string.compare("epsDisp")) {
										cdcParams.epsDisp = value;
									}
									else {
										if (!string.compare("t0")) {
											t0 = value;
										}
										else {
											if (!string.compare("alpha")) {
												alpha = value;
											}
											else {
												if (!string.compare("sigma")) {
													sigma = value;
												}
												else {
													if (!string.compare("lambda0")) {
														lambda0 = value;
													}
													else {
														if (!string.compare("gamma0")) {
															gamma0 = value;
														}
														else {
															if (!string.compare("qCdc")) {
																qCdc = value;
															}
															else {
																if (!string.compare("qSG")) {
																	qSG = value;
																}
																else {
																	if (!string.compare("nIter")) {
																		nIter = value;
																	}
																	else {
																		if (!string.compare("nTemps")) {
																			nTemps = value;
																		}
																		else {
																			if (!string.compare("tau_k")) {
																				tau_k = value;
																			}
																			else {
																				if (!string.compare("tau_d")) {
																					tau_d = value;
																				}
																				else {
																					if (!string.compare("deltaA")) {
																						deltaA = value;
																					}
																					else {
																						if (!string.compare("innerScale")) {
																							innerScale = value;
																						}
																						else {
																							if (!string.compare("outerScale")) {
																								outerScale = value;
																							}
																							else {
																								if (!string.compare("deltaAvg")) {
																									deltaAvg = value;
																								}
																								else {
																									std::cout << "unrecognized parameter: " << string << std::endl;
																								}
																							}
																						}
																					}
																				}
																			}
																		}
																	}
																}
															}
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
		
		static std::vector<std::string> parameterList() {
			std::vector<std::string> parameters = CDCParameters::parameterList();
			
			parameters.push_back("t0");
			parameters.push_back("alpha");
			parameters.push_back("sigma");
			parameters.push_back("lambda0");
			parameters.push_back("gamma0");
			parameters.push_back("tau_k");
			parameters.push_back("tau_d");
			parameters.push_back("innerScale");
			parameters.push_back("outerScale");
			parameters.push_back("nIter");
			parameters.push_back("nTemps");
			parameters.push_back("qCdc");
			parameters.push_back("qSG");
			parameters.push_back("deltaA");
			parameters.push_back("deltaAvg");
			return parameters;
		}
	};

	struct Settings {
		bool occlusionAware = true;
		bool useCDC = true;
		bool useSP = true;
		bool useCDCHeuristic = true;
		bool useSPHeuristic = true;
		bool useSDHeuristic = true;

		static std::vector<std::string> settingList() {
			std::vector<std::string> settings;

			settings.push_back("occlusionAware");
			settings.push_back("useCDC");
			settings.push_back("useSP");
			settings.push_back("useCDCHeuristic");
			settings.push_back("useSPHeuristic");
			settings.push_back("useSDHeuristic");
			
			return settings;
		}

		void changeValueFromString(std::string setting, bool value) {
			if (!setting.compare("occlusionAware")) {
				occlusionAware = value;
			}
			else {
				if (!setting.compare("useCDC")) {
					useCDC = value;
				}
				else {
					if (!setting.compare("useSP")) {
						useSP = value;
					}
					else {
						if (!setting.compare("useCDCHeuristic")) {
							useCDCHeuristic = value;
						}
						else {
							if (!setting.compare("useSPHeuristic")) {
								useSPHeuristic = value;
							}
							else {
								if (!setting.compare("useSDHeuristic")) {
									useSDHeuristic = value;
								}
								else {
									std::cerr << setting << " is not a recognize setting." << std::endl;
								}
							}
						}
					}
				}
			}			
		}
	};
}

