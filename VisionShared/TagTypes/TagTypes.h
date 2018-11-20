/*
 * TagTypes.h
 *
 *  Created on: Jun 10, 2016
 *      Author: Kobus
 */

#ifndef TAGTYPES_H_
#define TAGTYPES_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum TagType
{
	None = 0,
	LowSpeed,
	highSpeed,
	MedSpeed,
	Loco,
	Battery,
	NoGo,
	Person,
	Explosives,
	VentDoor,
	MenAtWork ,
	HaulTruckRigid,
	HaulTruckADT,
	WaterBowserRigid,
	WaterBowserADT,
	ExcavatorStandard,
	ExcavatorFaceShovel,
	ExcavatorRopeShovel,
	Dragline,
	FrontEndLoader,
	DozerWheel,
	DozerTrack,
	LightDutyVehicle,
	DrillRig,
	Grader,
	TractorLoaderBackhoe,
	Portable_unit,
	Side_tipper,
	Tractor,
	Bucket_wheel_excavator,
	Skid_loader,
	Fork_lift,
	Compact_roller,
	Sweeper,
	Locomotive_guard_car,
	Monorail_locomotive,
	Roof_bolter,
	Pipe_carrier,
	Reclaimer,
	Traffic_system_healthy,
	Traffic_system_faulty,
	Face_drill,
	Hammer_breaker,
	Cherry_picker,
	Winch_controller,
	Loading_Box,
	Battery_Bay,
	Workshop,
	Tips,
	Bend,
	Waiting_Point,
	Winch_scraper,
	Rail_bound_best_station,
	Trackless_test_station,
	Pedestrian_test_station,
	Rail_bound_test_point,
	Trackless_test_point,
	Pedestrian_test_point,
	Wetcreter,
	Water_Cannon,

} _TagType;

typedef enum GroupNames
{
	Haul = 1,
	Transport =1,
	Excavate,
	Loading,
	Surface_Forming,
	Rail_Bound_Equipment,
	Versatile,
	Purpose_Built,
	Pedestrians,
	System_Specific,

} _GroupNames;

#ifdef __cplusplus
}
#endif

#endif /* TAGTYPES_H_ */
