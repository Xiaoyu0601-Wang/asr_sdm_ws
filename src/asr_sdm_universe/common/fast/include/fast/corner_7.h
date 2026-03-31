#pragma once

#include <fast/faster_corner_utilities.h>

namespace fast {

template <class C>
inline bool is_corner_7(const unsigned char* p, const int w, const int barrier)
{
  const int cb = *p + barrier;
  const int c_b = *p - barrier;

  const int pixel[16] = {
    0 + w * 3,
    1 + w * 3,
    2 + w * 2,
    3 + w * 1,
    3 + w * 0,
    3 + w * -1,
    2 + w * -2,
    1 + w * -3,
    0 + w * -3,
    -1 + w * -3,
    -2 + w * -2,
    -3 + w * -1,
    -3 + w * 0,
    -3 + w * 1,
    -2 + w * 2,
    -1 + w * 3,
  };


			if(*(p + pixel[0]) > cb)
				if(*(p + pixel[8]) > cb)
					if(*(p + 3) > cb)
						if(*(p + pixel[5]) > cb)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[1]) > cb)
											return true;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[7]) > cb)
												return true;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else if(*(p + pixel[9]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	if(*(p + pixel[15]) < c_b)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[7]) > cb)
											return true;
										else if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else if(*(p + pixel[9]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	if(*(p + pixel[15]) < c_b)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else if(*(p + pixel[9]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																if(*(p + pixel[10]) > cb)
																	return true;
																else if(*(p + pixel[1]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else if(*(p + pixel[1]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[10]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else if(*(p + pixel[1]) > cb)
															if(*(p + pixel[2]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else if(*(p + pixel[1]) > cb)
															if(*(p + pixel[2]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[2]) > cb)
																return true;
															else if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[2]) > cb)
															return true;
														else if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																if(*(p + pixel[2]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[15]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else if(*(p + pixel[11]) < c_b)
														if(*(p + pixel[1]) > cb)
															if(*(p + pixel[2]) > cb)
																if(*(p + pixel[14]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[2]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[1]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else if(*(p + pixel[1]) > cb)
														if(*(p + pixel[2]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else if(*(p + pixel[11]) < c_b)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[2]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[2]) > cb)
															return true;
														else if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																if(*(p + pixel[11]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[15]) < c_b)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											return true;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																if(*(p + pixel[11]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[7]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[1]) > cb)
														return true;
													else if(*(p + pixel[10]) > cb)
														return true;
													else
														return false;
												else if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[9]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[9]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																if(*(p + pixel[11]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[15]) > cb)
											return true;
										else if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else if(*(p + pixel[15]) < c_b)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[11]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else if(*(p + pixel[9]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[9]) > cb)
													return true;
												else if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else if(*(p + pixel[9]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[7]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[9]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[11]) > cb)
								if(*(p + pixel[7]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + -3) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[9]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[6]) > cb)
											if(*(p + pixel[9]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + 3) < c_b)
						if(*(p + pixel[11]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else if(*(p + pixel[9]) > cb)
												return true;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															if(*(p + pixel[5]) < c_b)
																if(*(p + pixel[6]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															if(*(p + pixel[5]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[9]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + -3) > cb)
													return true;
												else if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[13]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + -3) > cb)
												return true;
											else if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													return true;
												else if(*(p + -3) > cb)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[13]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													return true;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															if(*(p + pixel[5]) < c_b)
																if(*(p + pixel[6]) < c_b)
																	if(*(p + pixel[7]) < c_b)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[11]) > cb)
						if(*(p + pixel[13]) > cb)
							if(*(p + -3) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else if(*(p + pixel[15]) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[15]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) < c_b)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																if(*(p + pixel[10]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[9]) > cb)
												return true;
											else if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[3]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[2]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) < c_b)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[7]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[6]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[7]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[11]) < c_b)
						if(*(p + pixel[14]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[13]) > cb)
						if(*(p + pixel[2]) > cb)
							if(*(p + -3) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[1]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) < c_b)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[1]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[8]) < c_b)
					if(*(p + pixel[9]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[2]) > cb)
												return true;
											else if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[11]) > cb)
													return true;
												else if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													return true;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[9]) < c_b)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[5]) > cb)
														return true;
													else if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															if(*(p + -3) < c_b)
																if(*(p + pixel[13]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[5]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[14]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[6]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														return true;
													else if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + 3) > cb)
															return true;
														else if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else if(*(p + pixel[1]) > cb)
																if(*(p + pixel[2]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else if(*(p + pixel[1]) > cb)
															if(*(p + pixel[2]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[11]) > cb)
														return true;
													else if(*(p + pixel[1]) > cb)
														if(*(p + pixel[2]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[15]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[11]) > cb)
													return true;
												else if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[15]) > cb)
													if(*(p + pixel[1]) > cb)
														return true;
													else if(*(p + pixel[1]) < c_b)
														if(*(p + pixel[11]) < c_b)
															if(*(p + -3) < c_b)
																if(*(p + pixel[13]) < c_b)
																	return true;
																else if(*(p + pixel[6]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[11]) < c_b)
															if(*(p + -3) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[11]) < c_b)
															if(*(p + -3) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[15]) < c_b)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[11]) < c_b)
																if(*(p + -3) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + pixel[11]) < c_b)
															if(*(p + -3) < c_b)
																if(*(p + pixel[13]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																return true;
															else if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + -3) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[1]) > cb)
																if(*(p + pixel[6]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + pixel[11]) < c_b)
															return true;
														else if(*(p + pixel[6]) > cb)
															if(*(p + pixel[1]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[6]) > cb)
														if(*(p + pixel[1]) > cb)
															return true;
														else
															return false;
													else if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[6]) > cb)
													if(*(p + pixel[1]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + -3) > cb)
														if(*(p + pixel[1]) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + -3) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[13]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[1]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) < c_b)
												if(*(p + -3) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														return true;
													else if(*(p + pixel[6]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + -3) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[6]) < c_b)
														return true;
													else if(*(p + pixel[13]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[13]) < c_b)
													return true;
												else if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[2]) > cb)
																if(*(p + pixel[3]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + -3) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + -3) < c_b)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[6]) < c_b)
														return true;
													else if(*(p + pixel[1]) > cb)
														if(*(p + pixel[2]) > cb)
															if(*(p + pixel[3]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[13]) < c_b)
													return true;
												else if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[3]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[2]) > cb)
															return true;
														else if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[6]) < c_b)
													return true;
												else if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[13]) < c_b)
												return true;
											else if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[2]) > cb)
																if(*(p + pixel[3]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + pixel[2]) > cb)
															if(*(p + pixel[3]) > cb)
																if(*(p + 3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + -3) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else if(*(p + pixel[3]) > cb)
																	if(*(p + 3) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + -3) < c_b)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[2]) > cb)
															if(*(p + pixel[3]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[13]) < c_b)
													return true;
												else if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + 3) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else if(*(p + 3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[3]) > cb)
																return true;
															else
																return false;
														else if(*(p + pixel[3]) > cb)
															if(*(p + 3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + 3) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[2]) > cb)
																if(*(p + pixel[3]) > cb)
																	return true;
																else if(*(p + -3) > cb)
																	if(*(p + pixel[13]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else if(*(p + -3) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) < c_b)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[3]) > cb)
																return true;
															else if(*(p + -3) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											return true;
										else if(*(p + pixel[11]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[2]) > cb)
																if(*(p + pixel[3]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											return true;
										else if(*(p + pixel[13]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + -3) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + -3) > cb)
															return true;
														else if(*(p + pixel[3]) > cb)
															return true;
														else
															return false;
													else if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) < c_b)
											if(*(p + pixel[11]) < c_b)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else if(*(p + pixel[11]) > cb)
														return true;
													else
														return false;
												else if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else if(*(p + 3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[6]) < c_b)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[13]) < c_b)
											return true;
										else if(*(p + pixel[6]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) < c_b)
											return true;
										else if(*(p + pixel[14]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + 3) > cb)
															return true;
														else if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[1]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[5]) > cb)
													return true;
												else if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[2]) > cb)
													return true;
												else if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[15]) < c_b)
									if(*(p + 3) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[2]) > cb)
															return true;
														else if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + 3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																if(*(p + pixel[11]) > cb)
																	return true;
																else if(*(p + pixel[2]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											return true;
										else if(*(p + pixel[11]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else if(*(p + pixel[2]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else if(*(p + pixel[11]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else if(*(p + pixel[14]) > cb)
												return true;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[15]) < c_b)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[13]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[13]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[11]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												if(*(p + pixel[1]) > cb)
													return true;
												else if(*(p + pixel[10]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) > cb)
							if(*(p + pixel[11]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[1]) > cb)
												return true;
											else if(*(p + pixel[10]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[15]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[11]) > cb)
											return true;
										else if(*(p + pixel[2]) > cb)
											return true;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) < c_b)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) > cb)
										if(*(p + pixel[2]) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + 3) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[2]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												return true;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[15]) < c_b)
						if(*(p + pixel[2]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[2]) > cb)
						if(*(p + pixel[6]) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + 3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[2]) < c_b)
						if(*(p + pixel[6]) < c_b)
							if(*(p + 3) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[2]) > cb)
					if(*(p + pixel[3]) > cb)
						if(*(p + pixel[5]) > cb)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + 3) > cb)
										return true;
									else if(*(p + pixel[13]) > cb)
										if(*(p + pixel[14]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + 3) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[6]) > cb)
											return true;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + 3) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[13]) > cb)
											return true;
										else if(*(p + 3) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[15]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) > cb)
										return true;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + 3) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[15]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[3]) < c_b)
						if(*(p + -3) > cb)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[1]) > cb)
											return true;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + -3) < c_b)
							if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + -3) > cb)
						if(*(p + pixel[14]) > cb)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[1]) > cb)
										return true;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + -3) < c_b)
						if(*(p + pixel[15]) < c_b)
							if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[2]) < c_b)
					if(*(p + pixel[15]) > cb)
						if(*(p + pixel[11]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[10]) > cb)
											return true;
										else if(*(p + pixel[1]) > cb)
											return true;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[15]) < c_b)
						if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[9]) < c_b)
							if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[11]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[1]) < c_b)
						if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) > cb)
					if(*(p + pixel[14]) > cb)
						if(*(p + pixel[13]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + -3) > cb)
										return true;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[10]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[15]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[15]) > cb)
									if(*(p + -3) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) < c_b)
					if(*(p + pixel[15]) < c_b)
						if(*(p + pixel[9]) < c_b)
							if(*(p + pixel[10]) < c_b)
								if(*(p + -3) < c_b)
									if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[14]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[0]) < c_b)
				if(*(p + pixel[8]) > cb)
					if(*(p + pixel[15]) > cb)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															if(*(p + 3) < c_b)
																if(*(p + pixel[5]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[13]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + 3) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												return true;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[9]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[15]) < c_b)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[6]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[3]) > cb)
													return true;
												else if(*(p + pixel[10]) > cb)
													return true;
												else if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[11]) > cb)
												if(*(p + pixel[10]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													return true;
												else if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														if(*(p + -3) < c_b)
															if(*(p + pixel[13]) < c_b)
																if(*(p + pixel[14]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[11]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[3]) > cb)
														return true;
													else if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[3]) > cb)
															return true;
														else if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[9]) > cb)
																return true;
															else if(*(p + pixel[3]) > cb)
																if(*(p + 3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + pixel[3]) > cb)
															if(*(p + 3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + -3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												return true;
											else if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[3]) > cb)
																return true;
															else if(*(p + pixel[9]) > cb)
																if(*(p + pixel[10]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																if(*(p + pixel[10]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[5]) > cb)
														return true;
													else if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																if(*(p + -3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[5]) > cb)
															return true;
														else if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															return true;
														else if(*(p + pixel[5]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[3]) > cb)
													return true;
												else if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[11]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + 3) > cb)
															return true;
														else if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + 3) > cb)
																if(*(p + pixel[5]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + 3) > cb)
															if(*(p + pixel[5]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												return true;
											else if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + 3) > cb)
																return true;
															else if(*(p + pixel[10]) > cb)
																if(*(p + pixel[11]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + 3) > cb)
															return true;
														else if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + 3) > cb)
														return true;
													else if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + 3) > cb)
																return true;
															else if(*(p + pixel[11]) > cb)
																return true;
															else
																return false;
														else if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[11]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[13]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[5]) > cb)
																if(*(p + pixel[6]) > cb)
																	return true;
																else if(*(p + pixel[11]) > cb)
																	if(*(p + -3) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else if(*(p + pixel[11]) > cb)
																if(*(p + -3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + 3) < c_b)
												return true;
											else if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[5]) > cb)
																if(*(p + pixel[6]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[13]) < c_b)
											return true;
										else if(*(p + 3) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[5]) > cb)
																return true;
															else if(*(p + pixel[11]) > cb)
																if(*(p + -3) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											return true;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																return true;
															else if(*(p + pixel[5]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + 3) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + 3) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + -3) > cb)
																if(*(p + pixel[13]) > cb)
																	return true;
																else if(*(p + pixel[6]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											return true;
										else if(*(p + pixel[10]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + -3) > cb)
															if(*(p + pixel[13]) > cb)
																return true;
															else if(*(p + pixel[6]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[11]) > cb)
														return true;
													else if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[13]) > cb)
													if(*(p + pixel[11]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[14]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else if(*(p + 3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[13]) < c_b)
											return true;
										else if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																return true;
															else if(*(p + 3) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[11]) > cb)
															return true;
														else if(*(p + 3) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else if(*(p + 3) > cb)
													if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[13]) > cb)
													if(*(p + -3) > cb)
														return true;
													else
														return false;
												else if(*(p + pixel[6]) > cb)
													if(*(p + -3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[10]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[13]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[13]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[3]) > cb)
															return true;
														else if(*(p + pixel[10]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + -3) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[3]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + -3) < c_b)
										if(*(p + pixel[14]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																if(*(p + pixel[10]) > cb)
																	return true;
																else if(*(p + pixel[3]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[14]) < c_b)
											return true;
										else if(*(p + pixel[6]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															if(*(p + pixel[10]) > cb)
																return true;
															else if(*(p + pixel[3]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															return true;
														else if(*(p + pixel[3]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[3]) > cb)
														return true;
													else if(*(p + pixel[10]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + 3) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else if(*(p + pixel[3]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												return true;
											else if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[5]) > cb)
													return true;
												else if(*(p + -3) > cb)
													return true;
												else
													return false;
											else if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												return true;
											else if(*(p + pixel[5]) > cb)
												return true;
											else
												return false;
										else if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[11]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[14]) < c_b)
										if(*(p + -3) < c_b)
											return true;
										else if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															if(*(p + pixel[9]) > cb)
																return true;
															else if(*(p + pixel[2]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														if(*(p + pixel[9]) > cb)
															return true;
														else if(*(p + pixel[2]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													if(*(p + pixel[2]) > cb)
														return true;
													else if(*(p + pixel[9]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[2]) > cb)
													return true;
												else if(*(p + pixel[9]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[9]) > cb)
												return true;
											else if(*(p + pixel[2]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[9]) > cb)
						if(*(p + pixel[10]) > cb)
							if(*(p + pixel[11]) > cb)
								if(*(p + pixel[7]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											return true;
										else if(*(p + pixel[6]) > cb)
											return true;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + 3) < c_b)
															if(*(p + pixel[5]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[14]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												return true;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + 3) < c_b)
															if(*(p + pixel[5]) < c_b)
																if(*(p + pixel[6]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											return true;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[7]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) > cb)
								if(*(p + 3) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[6]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[3]) > cb)
								if(*(p + 3) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[9]) < c_b)
						if(*(p + 3) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[6]) > cb)
						if(*(p + pixel[2]) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + 3) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[6]) < c_b)
						if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[8]) < c_b)
					if(*(p + -3) > cb)
						if(*(p + 3) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													return true;
												else if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[5]) > cb)
																if(*(p + pixel[6]) > cb)
																	if(*(p + pixel[7]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[3]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[11]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																if(*(p + pixel[11]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													return true;
												else if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																if(*(p + pixel[10]) < c_b)
																	if(*(p + pixel[11]) < c_b)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																if(*(p + pixel[11]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[11]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) < c_b)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[10]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[9]) > cb)
													if(*(p + pixel[10]) > cb)
														if(*(p + pixel[11]) > cb)
															if(*(p + pixel[13]) > cb)
																if(*(p + pixel[14]) > cb)
																	if(*(p + pixel[15]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[9]) < c_b)
													return true;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[15]) > cb)
													if(*(p + pixel[9]) > cb)
														if(*(p + pixel[10]) > cb)
															if(*(p + pixel[11]) > cb)
																if(*(p + pixel[13]) > cb)
																	if(*(p + pixel[14]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else if(*(p + pixel[1]) < c_b)
												return true;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[1]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[2]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[9]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[11]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												return true;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																if(*(p + pixel[11]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[11]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + -3) < c_b)
						if(*(p + pixel[13]) > cb)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[15]) < c_b)
													if(*(p + pixel[5]) < c_b)
														return true;
													else if(*(p + pixel[14]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												return true;
											else if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[15]) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) < c_b)
							if(*(p + pixel[11]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[3]) < c_b)
															return true;
														else if(*(p + pixel[10]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																if(*(p + pixel[10]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[10]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												return true;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[2]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																if(*(p + pixel[9]) < c_b)
																	return true;
																else if(*(p + pixel[2]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[2]) < c_b)
																return true;
															else if(*(p + pixel[7]) < c_b)
																if(*(p + pixel[9]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																return true;
															else if(*(p + pixel[2]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[15]) < c_b)
												return true;
											else if(*(p + pixel[2]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[5]) > cb)
																if(*(p + pixel[6]) > cb)
																	if(*(p + pixel[7]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else if(*(p + pixel[1]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											return true;
										else if(*(p + pixel[15]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[1]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[5]) > cb)
																if(*(p + pixel[6]) > cb)
																	if(*(p + pixel[7]) > cb)
																		return true;
																	else
																		return false;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else if(*(p + pixel[1]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											return true;
										else if(*(p + pixel[6]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + 3) > cb)
															if(*(p + pixel[5]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + 3) < c_b)
															if(*(p + pixel[1]) < c_b)
																return true;
															else if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																return true;
															else if(*(p + pixel[2]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[2]) < c_b)
																return true;
															else if(*(p + pixel[7]) < c_b)
																if(*(p + pixel[9]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											return true;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[2]) > cb)
															if(*(p + pixel[7]) < c_b)
																if(*(p + pixel[9]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else if(*(p + pixel[2]) < c_b)
															return true;
														else if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[2]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + 3) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + 3) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[2]) > cb)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[9]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else if(*(p + pixel[1]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											return true;
										else if(*(p + 3) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[1]) < c_b)
														if(*(p + pixel[2]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[2]) < c_b)
														return true;
													else if(*(p + pixel[9]) < c_b)
														return true;
													else
														return false;
												else if(*(p + pixel[15]) < c_b)
													if(*(p + pixel[1]) < c_b)
														if(*(p + pixel[2]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) > cb)
								if(*(p + pixel[6]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[10]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[15]) > cb)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[1]) < c_b)
														return true;
													else if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[15]) < c_b)
									if(*(p + pixel[1]) > cb)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[3]) < c_b)
															return true;
														else if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[14]) > cb)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																if(*(p + pixel[10]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[14]) < c_b)
											return true;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																if(*(p + pixel[10]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else if(*(p + pixel[3]) < c_b)
													return true;
												else if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															if(*(p + pixel[9]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + 3) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[3]) < c_b)
														return true;
													else if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[9]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[1]) < c_b)
													return true;
												else if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + 3) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[10]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[6]) > cb)
							if(*(p + pixel[15]) < c_b)
								if(*(p + 3) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else if(*(p + pixel[5]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[6]) < c_b)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[3]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[1]) < c_b)
													return true;
												else if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[11]) > cb)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													return true;
												else if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) < c_b)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															return true;
														else if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															return true;
														else if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											return true;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[1]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else if(*(p + pixel[5]) < c_b)
														return true;
													else if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															if(*(p + pixel[1]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[5]) < c_b)
														return true;
													else if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[7]) > cb)
												if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[2]) < c_b)
														if(*(p + pixel[3]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else if(*(p + pixel[7]) < c_b)
												return true;
											else if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[1]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[1]) < c_b)
													return true;
												else if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[1]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else if(*(p + pixel[1]) < c_b)
												return true;
											else
												return false;
										else if(*(p + pixel[14]) < c_b)
											if(*(p + pixel[15]) < c_b)
												if(*(p + pixel[1]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[5]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) < c_b)
							if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[1]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[1]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + 3) > cb)
						if(*(p + pixel[3]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												return true;
											else if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																if(*(p + pixel[11]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[11]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + pixel[11]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[11]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + 3) < c_b)
						if(*(p + pixel[3]) > cb)
							if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[15]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else if(*(p + pixel[5]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else if(*(p + pixel[1]) < c_b)
											return true;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[1]) < c_b)
											return true;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[1]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[3]) > cb)
						if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[3]) < c_b)
						if(*(p + pixel[13]) > cb)
							if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) < c_b)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + pixel[11]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[14]) > cb)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[10]) < c_b)
														if(*(p + pixel[11]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[11]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[9]) < c_b)
															if(*(p + pixel[10]) < c_b)
																if(*(p + pixel[11]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											return true;
										else if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[10]) < c_b)
															if(*(p + pixel[9]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[9]) < c_b)
													if(*(p + pixel[7]) < c_b)
														if(*(p + pixel[10]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													if(*(p + pixel[9]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[7]) < c_b)
											if(*(p + pixel[9]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[11]) < c_b)
						if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + -3) > cb)
					if(*(p + pixel[3]) > cb)
						if(*(p + pixel[15]) > cb)
							if(*(p + pixel[14]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + 3) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[3]) < c_b)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + 3) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[14]) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[15]) > cb)
											if(*(p + pixel[6]) < c_b)
												return true;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + pixel[11]) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[15]) < c_b)
											return true;
										else if(*(p + pixel[6]) < c_b)
											return true;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[15]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) > cb)
									if(*(p + pixel[15]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + pixel[11]) > cb)
													if(*(p + pixel[14]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[14]) < c_b)
										if(*(p + pixel[15]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[15]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[15]) > cb)
						if(*(p + pixel[9]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[14]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + -3) < c_b)
					if(*(p + pixel[14]) > cb)
						if(*(p + pixel[5]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[14]) < c_b)
						if(*(p + pixel[2]) > cb)
							if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[10]) < c_b)
												return true;
											else if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											return true;
										else if(*(p + pixel[10]) < c_b)
											return true;
										else
											return false;
									else if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[1]) < c_b)
										return true;
									else if(*(p + pixel[10]) < c_b)
										if(*(p + pixel[11]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + 3) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[1]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[3]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[15]) < c_b)
										return true;
									else
										return false;
								else if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[15]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[5]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[5]) < c_b)
						if(*(p + pixel[2]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[15]) > cb)
									if(*(p + pixel[1]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[15]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + 3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + 3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[3]) > cb)
					if(*(p + pixel[1]) > cb)
						if(*(p + pixel[6]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + 3) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[3]) < c_b)
					if(*(p + pixel[2]) < c_b)
						if(*(p + pixel[5]) > cb)
							if(*(p + pixel[13]) > cb)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[15]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + 3) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[15]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + pixel[15]) > cb)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + 3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + 3) < c_b)
										return true;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[14]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + 3) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) > cb)
							if(*(p + pixel[14]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[15]) < c_b)
										if(*(p + pixel[1]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) < c_b)
							if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[15]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + 3) < c_b)
							if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[15]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[8]) > cb)
				if(*(p + 3) > cb)
					if(*(p + pixel[6]) > cb)
						if(*(p + pixel[10]) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[9]) > cb)
										return true;
									else if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[9]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + -3) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[9]) > cb)
											if(*(p + pixel[13]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[9]) > cb)
											return true;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[2]) > cb)
												return true;
											else if(*(p + pixel[11]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[2]) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[11]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[7]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[7]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) > cb)
								if(*(p + pixel[7]) > cb)
									if(*(p + pixel[5]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[6]) < c_b)
						if(*(p + pixel[11]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[7]) > cb)
												return true;
											else if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[13]) > cb)
						if(*(p + pixel[10]) > cb)
							if(*(p + pixel[11]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[13]) < c_b)
						if(*(p + pixel[9]) < c_b)
							if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[11]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[14]) < c_b)
											if(*(p + pixel[15]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + 3) < c_b)
					if(*(p + pixel[11]) > cb)
						if(*(p + pixel[9]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + -3) > cb)
											return true;
										else if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else if(*(p + pixel[1]) < c_b)
												if(*(p + pixel[2]) < c_b)
													if(*(p + pixel[3]) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[5]) > cb)
												return true;
											else if(*(p + -3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + -3) > cb)
											return true;
										else if(*(p + pixel[5]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[9]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[11]) < c_b)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												return true;
											else if(*(p + pixel[9]) < c_b)
												if(*(p + pixel[10]) < c_b)
													if(*(p + -3) < c_b)
														if(*(p + pixel[13]) < c_b)
															if(*(p + pixel[14]) < c_b)
																if(*(p + pixel[15]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) < c_b)
											if(*(p + pixel[10]) < c_b)
												if(*(p + -3) < c_b)
													if(*(p + pixel[13]) < c_b)
														if(*(p + pixel[14]) < c_b)
															if(*(p + pixel[15]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[10]) < c_b)
											if(*(p + -3) < c_b)
												if(*(p + pixel[13]) < c_b)
													if(*(p + pixel[14]) < c_b)
														if(*(p + pixel[15]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													if(*(p + pixel[15]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												if(*(p + pixel[15]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[9]) < c_b)
							if(*(p + pixel[15]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[13]) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[7]) < c_b)
						if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) > cb)
					if(*(p + pixel[10]) > cb)
						if(*(p + pixel[5]) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[6]) > cb)
										return true;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[9]) > cb)
										if(*(p + -3) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[5]) < c_b)
							if(*(p + -3) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[7]) > cb)
										if(*(p + pixel[13]) > cb)
											return true;
										else if(*(p + pixel[6]) > cb)
											return true;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[14]) > cb)
										if(*(p + pixel[13]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + -3) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[9]) > cb)
										return true;
									else
										return false;
								else if(*(p + pixel[6]) > cb)
									if(*(p + pixel[9]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[14]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[9]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) < c_b)
					if(*(p + pixel[9]) < c_b)
						if(*(p + pixel[15]) < c_b)
							if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[10]) < c_b)
										if(*(p + -3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[8]) < c_b)
				if(*(p + 3) > cb)
					if(*(p + pixel[11]) > cb)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												return true;
											else if(*(p + pixel[9]) > cb)
												if(*(p + pixel[10]) > cb)
													if(*(p + -3) > cb)
														if(*(p + pixel[13]) > cb)
															if(*(p + pixel[14]) > cb)
																if(*(p + pixel[15]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[10]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[10]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[10]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[9]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[11]) < c_b)
						if(*(p + pixel[10]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + -3) > cb)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + -3) < c_b)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[6]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[6]) < c_b)
											if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[14]) < c_b)
												return true;
											else if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[7]) < c_b)
											return true;
										else if(*(p + pixel[14]) < c_b)
											return true;
										else
											return false;
									else if(*(p + pixel[6]) > cb)
										if(*(p + pixel[7]) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[5]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) > cb)
								if(*(p + pixel[7]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[6]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[7]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[6]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + 3) < c_b)
					if(*(p + pixel[6]) > cb)
						if(*(p + pixel[11]) > cb)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[10]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[11]) < c_b)
							if(*(p + pixel[13]) < c_b)
								if(*(p + pixel[10]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[14]) < c_b)
												return true;
											else if(*(p + pixel[7]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[6]) < c_b)
						if(*(p + pixel[10]) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else if(*(p + pixel[9]) > cb)
											if(*(p + pixel[11]) > cb)
												if(*(p + -3) > cb)
													if(*(p + pixel[13]) > cb)
														if(*(p + pixel[14]) > cb)
															if(*(p + pixel[15]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) > cb)
										if(*(p + pixel[11]) > cb)
											if(*(p + -3) > cb)
												if(*(p + pixel[13]) > cb)
													if(*(p + pixel[14]) > cb)
														if(*(p + pixel[15]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[9]) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + -3) > cb)
											if(*(p + pixel[13]) > cb)
												if(*(p + pixel[14]) > cb)
													if(*(p + pixel[15]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) > cb)
								if(*(p + -3) > cb)
									if(*(p + pixel[11]) > cb)
										if(*(p + pixel[13]) > cb)
											if(*(p + pixel[14]) > cb)
												if(*(p + pixel[15]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + -3) < c_b)
											if(*(p + pixel[13]) < c_b)
												if(*(p + pixel[14]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + -3) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[9]) < c_b)
										return true;
									else if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + -3) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + pixel[11]) < c_b)
											if(*(p + pixel[13]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[9]) > cb)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[9]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[7]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[5]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[13]) > cb)
						if(*(p + pixel[9]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[11]) > cb)
									if(*(p + -3) > cb)
										if(*(p + pixel[14]) > cb)
											if(*(p + pixel[15]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[13]) < c_b)
						if(*(p + pixel[10]) < c_b)
							if(*(p + pixel[11]) < c_b)
								if(*(p + pixel[14]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + -3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[9]) < c_b)
										if(*(p + -3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) > cb)
					if(*(p + pixel[9]) > cb)
						if(*(p + -3) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[14]) > cb)
										if(*(p + pixel[15]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[11]) < c_b)
					if(*(p + pixel[10]) < c_b)
						if(*(p + pixel[13]) > cb)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											if(*(p + -3) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else if(*(p + -3) < c_b)
									if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[9]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[13]) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + -3) < c_b)
										return true;
									else if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[6]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[14]) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + -3) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[6]) < c_b)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[9]) < c_b)
									if(*(p + -3) < c_b)
										if(*(p + pixel[7]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[5]) < c_b)
								if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[9]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else if(*(p + -3) < c_b)
								if(*(p + pixel[9]) < c_b)
									if(*(p + pixel[7]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[9]) > cb)
				if(*(p + pixel[15]) > cb)
					if(*(p + -3) > cb)
						if(*(p + pixel[14]) > cb)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[13]) > cb)
									if(*(p + pixel[11]) > cb)
										return true;
									else if(*(p + pixel[11]) < c_b)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[1]) > cb)
												if(*(p + pixel[2]) > cb)
													if(*(p + pixel[3]) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[13]) < c_b)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + 3) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[6]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + -3) < c_b)
						if(*(p + pixel[3]) > cb)
							if(*(p + pixel[7]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[1]) > cb)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[6]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[1]) < c_b)
						if(*(p + 3) < c_b)
							if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[15]) < c_b)
					if(*(p + pixel[7]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[7]) < c_b)
						if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[1]) > cb)
					if(*(p + pixel[7]) > cb)
						if(*(p + 3) > cb)
							if(*(p + pixel[6]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[5]) > cb)
										if(*(p + pixel[3]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[1]) < c_b)
					if(*(p + pixel[7]) < c_b)
						if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + 3) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[9]) < c_b)
				if(*(p + pixel[15]) > cb)
					if(*(p + pixel[2]) > cb)
						if(*(p + pixel[7]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[3]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[5]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[2]) < c_b)
						if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + 3) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[15]) < c_b)
					if(*(p + -3) > cb)
						if(*(p + pixel[3]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[6]) > cb)
												if(*(p + pixel[7]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[3]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + 3) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[7]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + -3) < c_b)
						if(*(p + pixel[14]) > cb)
							if(*(p + pixel[1]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + 3) > cb)
											if(*(p + pixel[5]) > cb)
												if(*(p + pixel[6]) > cb)
													if(*(p + pixel[7]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[5]) < c_b)
												if(*(p + pixel[6]) < c_b)
													if(*(p + pixel[7]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[14]) < c_b)
							if(*(p + pixel[10]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														if(*(p + pixel[7]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[2]) < c_b)
										if(*(p + pixel[3]) < c_b)
											if(*(p + 3) < c_b)
												if(*(p + pixel[5]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + pixel[7]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[10]) < c_b)
								if(*(p + pixel[11]) > cb)
									if(*(p + pixel[1]) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[3]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															if(*(p + pixel[7]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[3]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[5]) < c_b)
														if(*(p + pixel[6]) < c_b)
															if(*(p + pixel[7]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[11]) < c_b)
									if(*(p + pixel[13]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																if(*(p + pixel[7]) > cb)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															if(*(p + pixel[6]) < c_b)
																if(*(p + pixel[7]) < c_b)
																	return true;
																else
																	return false;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[13]) < c_b)
										return true;
									else if(*(p + pixel[7]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + pixel[3]) > cb)
													if(*(p + 3) > cb)
														if(*(p + pixel[5]) > cb)
															if(*(p + pixel[6]) > cb)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else if(*(p + pixel[7]) < c_b)
										if(*(p + pixel[1]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + pixel[6]) < c_b)
														if(*(p + 3) < c_b)
															if(*(p + pixel[5]) < c_b)
																return true;
															else
																return false;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) > cb)
									if(*(p + pixel[3]) > cb)
										if(*(p + pixel[1]) > cb)
											if(*(p + pixel[2]) > cb)
												if(*(p + 3) > cb)
													if(*(p + pixel[5]) > cb)
														if(*(p + pixel[6]) > cb)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else if(*(p + pixel[7]) < c_b)
									if(*(p + pixel[1]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + pixel[6]) < c_b)
												if(*(p + pixel[3]) < c_b)
													if(*(p + 3) < c_b)
														if(*(p + pixel[5]) < c_b)
															return true;
														else
															return false;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[5]) > cb)
													if(*(p + pixel[6]) > cb)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else if(*(p + pixel[7]) < c_b)
								if(*(p + pixel[1]) < c_b)
									if(*(p + pixel[3]) < c_b)
										if(*(p + pixel[5]) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + 3) < c_b)
													if(*(p + pixel[6]) < c_b)
														return true;
													else
														return false;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[1]) > cb)
									if(*(p + pixel[2]) > cb)
										if(*(p + pixel[3]) > cb)
											if(*(p + 3) > cb)
												if(*(p + pixel[6]) > cb)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else if(*(p + pixel[7]) < c_b)
							if(*(p + pixel[1]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + 3) < c_b)
											if(*(p + pixel[2]) < c_b)
												if(*(p + pixel[5]) < c_b)
													return true;
												else
													return false;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[7]) > cb)
						if(*(p + pixel[1]) > cb)
							if(*(p + pixel[3]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[2]) > cb)
											if(*(p + pixel[6]) > cb)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else if(*(p + pixel[7]) < c_b)
						if(*(p + pixel[1]) < c_b)
							if(*(p + pixel[3]) < c_b)
								if(*(p + pixel[5]) < c_b)
									if(*(p + pixel[6]) < c_b)
										if(*(p + pixel[2]) < c_b)
											if(*(p + 3) < c_b)
												return true;
											else
												return false;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[1]) > cb)
					if(*(p + pixel[7]) > cb)
						if(*(p + pixel[3]) > cb)
							if(*(p + pixel[5]) > cb)
								if(*(p + pixel[2]) > cb)
									if(*(p + 3) > cb)
										if(*(p + pixel[6]) > cb)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else if(*(p + pixel[1]) < c_b)
					if(*(p + pixel[7]) < c_b)
						if(*(p + 3) < c_b)
							if(*(p + pixel[6]) < c_b)
								if(*(p + pixel[2]) < c_b)
									if(*(p + pixel[5]) < c_b)
										if(*(p + pixel[3]) < c_b)
											return true;
										else
											return false;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[7]) > cb)
				if(*(p + pixel[1]) > cb)
					if(*(p + 3) > cb)
						if(*(p + pixel[6]) > cb)
							if(*(p + pixel[2]) > cb)
								if(*(p + pixel[5]) > cb)
									if(*(p + pixel[3]) > cb)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else if(*(p + pixel[7]) < c_b)
				if(*(p + pixel[1]) < c_b)
					if(*(p + 3) < c_b)
						if(*(p + pixel[6]) < c_b)
							if(*(p + pixel[2]) < c_b)
								if(*(p + pixel[3]) < c_b)
									if(*(p + pixel[5]) < c_b)
										return true;
									else
										return false;
								else
									return false;
							else
								return false;
						else
							return false;
					else
						return false;
				else
					return false;
			else
				return false;


  return false;
}

} // namespace fast
