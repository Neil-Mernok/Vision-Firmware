/**
 * @author Kobus Goosen
 *
 * @section LICENSE
 *
 * Copyright (c) 2015 Mernok Elektronik
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *    Parameter Types Abstraction
 *
 */

#ifndef _PARAMVALUE_H_
#define _PARAMVALUE_H_

//#define USE_MAP
//#define USE_STRING

#define STR_MAX	15	/*max length of a string*/
#define max_params	35


#ifdef __cplusplus

/** Parameter class
 *
 */
struct ParamValue
{
	bool changed = false;

	/// This is the eeprom address of this parameter. 
	int address;
	int min = 0, max = 0x7FFFFFFF;
	bool isString = false;
	const char* name;
	
	// Empty constructor for uninitialised variables
	ParamValue()
	{
		address = 0;
		isString = false;
		min = 0;
		max = 0xFFFF;
		name = ""; 
		_value = 0;
	}
	
	// generic numeric constructor
	ParamValue(int adr, int Min, int value, int Max, const char* Name) :
			address(adr), min(Min) ,max(Max), name(Name) 
	{
		_value = value;
	}

	// string base constructor.
	ParamValue(int adr, const char* value, const char* Name) :
			address(adr), name(Name)
	{
		// create a new zero filled array. make it one byte longer to ensure strings terminated irrespective of eeprom. 
		_value = (uint32_t) new char[STR_MAX+1] {};
		memset(((char*)_value), 0, STR_MAX+1);
		strncpy(((char*)_value), value, STR_MAX);
		isString = true;
	}

	/****************************************************************************
	 * = Operator overloading for an ParamValue from some object				*
	 *																			*
	 * @param rhs object														*
	 * @return a reference on the ParamValue affected							*
	 ****************************************************************************/

	ParamValue& operator=(int rhs)
	{
		_value = rhs;
		return *this;
	}
	
	/// conversion for any general type. (sort of in place of a cast) 
	template<typename T>
	operator T&()
	{
		return (T&)_value;
	}
	/// conversion operator to get a string value
	operator char*() {return ((char*)_value);}		
	
	/// pointer to the data container.
	uint8_t* ptr() 
	{
		if(isString)
			return (uint8_t*)(_value);
		else
			return (uint8_t*)(&this->_value);
	}
	
	/// length of the item in question. 
	int len()
	{
		if(isString)
			return strnlen(((char*)_value), STR_MAX);
		else
			return 4;
	}
	
	/// max length of the item in question. 
	int maxlen()
	{
		if(isString)
			return STR_MAX;
		else return len();
	}
	
	void checkbounds()
	{
		if(_value > max) 
			_value = max;
		if(_value < min) 
			_value = min;
	}

//protected:
	// The actual number stored. can integer type or string type 
	int _value;

	// add the parameter to a parameter pointer list. 
	void add_to_list(ParamValue** list)
	{
		for (int i = 0; i < max_params; i++)
		{
			if (list[i] == NULL)
			{
				list[i] = this;
				break;
			}
		}
	}
};

#endif /* __cplusplus */


#endif // _PARAMVALUE_H_
