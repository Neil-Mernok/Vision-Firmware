package coza.mernok.visionlibs;

import android.util.SparseArray;

import com.mernok.visionlibs.R;


/**
 * Created by Kobus on 2015/11/06.
 */
public enum TagTypes {
    NONE(0, "none"),
	LOWSPEED(1, R. drawable.type_icon1, "Low speed zone"),
    HIGHSPEED(2, R.drawable.type_icon2, "High speed zone"),
    MEDSPEED(3, R.drawable.type_icon3, "Med speed zone"),
	LOCO(4, R.drawable.type_icon4, "Locomotive"),
    BATTERY(5, "Battery"),
	NOGO(6, "NoGo"),
	PERSON(7, R.drawable.type_icon7, "Person"),
    TRAFFIC_INTERSECTION(9, R.drawable.type_icon9, "Traffic Intersection"),
    EXPLOSIVE(11, R.drawable.type_icon11, "Explosive"),
    //TRACKLESS(12, R.drawable.img_haul_truck_adt, "Trackless"),
    VENTDOOR(13, R.drawable.type_icon13, "Ventilation door"),
	HAULAGE(14, "Haulage"),
    MEN_AT_WORK(15, R.drawable.type_icon7, "Men at work"),

    HAULTRUCK_RIGID(17, R.drawable.type_icon17, "Rigid Haul Truck"),
    HAULTRUCK_ADT(18, R.drawable.type_icon17, "Articulated Haul Truck"),
    WATERBOWSER_RIGID(19, "Rigid Water Bowser"),
    WATERBOWSER_ADT(20, "Articulated Water Bowser"),
    EXCAVATOR_STANDARD(21, R.drawable.type_icon21, "Excavator Standard"),
    EXCAVATOR_FACESHOVEL(22, R.drawable.type_icon21, "Excavator FaceShovel"),
    EXCAVATOR_ROPESHOVEL(23, R.drawable.type_icon21, "Excavator RopeShovel"),

	DRAGLINE(25, "Dragline"),
    FRONTEND_LOADER(26, R.drawable.type_icon26, "Front End Loader"),
    DOZER_WHEEL(27, R.drawable.type_icon27, "Wheeled Dozer"),
    DOZER_TRACK(28, R.drawable.type_icon28, "Track Dozer"),
    LDV(29, R.drawable.type_icon29, "Light Duty Vehicle"),
    DRILL_RIG(30, "Drilling Rig"),
    GRADER(31, R.drawable.type_icon31, "Grader"),
    TLB(33, R.drawable.type_icon33, "Backhoe Loader"),

	GO_LIGHT(49, R.drawable.type_icon9, "Go Light"),
	CAUTION_LIGHT(50, R.drawable.type_icon9, "Caution Light"),
	STOP_LIGHT(51, R.drawable.type_icon9, "Stop Light"),

	LOADING_BOX(57, "Loading Box"),
	BATTERY_BAY(58, "Battery Bay"),
	WORKSHOP(59, "Workshop"),
	TIPS(60, "Tips"),
	BEND(61, "Bend"),
	WAITING_POINT(62, R.drawable.type_icon7, "Waiting Point"),

    CIT_CANISTER(161, "CIT Canister"),
    CIT_DEPOTTAG(162, "CIT Depot"),
    CIT_SHOP(163,R.drawable.type_icon163, "CIT Shop"),
    CIT_BANK(164, "CIT Bank"),
    CIT_PERSON(165, R.drawable.type_icon7, "CIT Person");

    int code;
    String label;
	int icon =  R.drawable.type_icon0;
	
    private TagTypes(int CODE, int image, String LABEL) {
        code = CODE;
        label = LABEL;
	    icon = image;
    }

	private TagTypes(int CODE, String LABEL) {
		code = CODE;
		label = LABEL;
	}

    /**
     * A mapping between the integer code and its corresponding Status to
     * facilitate lookup by code.
     */
    static SparseArray<TagTypes> codeToPOD;

    public static TagTypes get_type(int i) {
        TagTypes P;
        if (codeToPOD == null) {
            codeToPOD = new SparseArray<TagTypes>();
            for (TagTypes p : values()) {
                codeToPOD.put(p.code, p);
            }
        }
        P = codeToPOD.get(i);
        if (P == null) {
            P = TagTypes.NONE;
	        P.label = String.valueOf(i);
	        P.code = i;
        }
        return P;
    }

    public static CharSequence[] getNames(){
        CharSequence[] names = new CharSequence[values().length];
        for (int i = 0; i< values().length; i++) {
            names[i] = values()[i].label;
        }
        return names;
    }

    public static CharSequence[] getCodes(){
        CharSequence[] codes = new CharSequence[values().length];
        for (int i = 0; i< values().length; i++) {
            codes[i] = Integer.toString(values()[i].code);
        }
        return codes;
    }

	public static int[] getIcons(){
		int[] icons = new int[values().length];
		for (int i = 0; i< values().length; i++) {
			icons[i] = values()[i].icon;
		}
		return icons;
	}

//    public int getColor() {
//        {
//            final int[] cols = { Color.BLACK, Color.RED, Color.GREEN, Color.YELLOW, Color.BLUE, Color.MAGENTA, Color.CYAN, Color.WHITE };
//            return cols[code & 0x7];
//        }
//    }

    public int getCode() {
        return code;
    }

	public int getIcon() {
		return icon;
	}

	@Override
	public String toString() {
        return label;
    }
}