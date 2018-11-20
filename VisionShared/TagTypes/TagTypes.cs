using System;
using System.ComponentModel;
using System.Drawing;


namespace Vision_Libs.Vision
{
    public enum TagType
    {
        [Description("none")]
        None = 0,
        [Description("Low speed zone")]
        LowSpeed = 1,
        [Description("High speed zone")]
        highSpeed = 2,
        [Description("Med speed zone")]
        MedSpeed = 3,
        [Description("Locomotive")]
        Loco = 4,
        [Description("Battery")]
        Battery = 5,
        [Description("NoGo")]
        NoGo = 6,
        [Description("Person")]
        Person = 7,
        [Description("Traffic Intersection")]
        TrafficIntersection = 9,
        [Description("Explosive")]
        Explosive = 11,
        //[Description("Trackless")]
        //Trackless = 12,
        [Description("Ventilation door")]
        VentDoor = 13,
        [Description("Haulage")]
        Haulage = 14,
        [Description("Men at work")]
        MenAtWork = 15,

        [Description("Rigid Haul Truck")]
        HaulTruckRigid = 17,
        [Description("Articulated Haul Truck")]
        HaulTruckADT = 18,
        [Description("Rigid Water Bowser")]
        WaterBowserRigid = 19,
        [Description("Articulated Water Bowser")]
        WaterBowserADT = 20,
        [Description("Excavator Standard")]
        ExcavatorStandard = 21,
        [Description("Excavator FaceShovel")]
        ExcavatorFaceShovel = 22,
        [Description("Excavator RopeShovel")]
        ExcavatorRopeShovel = 23,
        [Description("Dragline")]
        Dragline = 25,
        [Description("Front End Loader")]
        FrontEndLoader = 26,
        [Description("Wheeled Dozer")]
        DozerWheel = 27,
        [Description("Track Dozer")]
        DozerTrack = 28,
        [Description("Light Duty Vehicle")]
        LDV = 29,
        [Description("Drilling Rig")]
        DrillRig = 30,
        [Description("Grader")]
        Grader = 31,
        [Description("Backhoe Loader")]
        TLB = 33,

        [Description("Go Light")]
        GO_LIGHT = 49,
        [Description("Caution Light")]
        CAUTION_LIGHT = 50,
        [Description("Stop Light")]
        STOP_LIGHT = 51,

        [Description("Loading Box")]
        LOADING_BOX = 57,
        [Description("Battery Bay")]
        BATTERY_BAY = 58, 
        [Description("Workshop")]
        WORKSHOP = 59,
        [Description("Tips")]
        TIPS = 60, 
        [Description("Bend")]
        BEND = 61, 
        [Description("Waiting Point")]
        WAITING_POINT = 62,

        [Description("CIT Canister")]
        CIT_CanisterTag = 161,
        [Description("CIT Depot")]
        CIT_DepotTagTag = 162,
        [Description("CIT Shop")]
        CIT_ShopTag = 163,
        [Description("CIT Bank")]
        CIT_BankTag = 164,
        [Description("CIT Person")]
        CIT_PersonTag = 165,
    };   
}
