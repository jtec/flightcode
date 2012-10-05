/**
 * \file ParameterManager.cpp
 * \brief A ParameterManager makes all onboard parameter accessible by one single
 *        interface. Other UAVComponents can check in their parameters, making them accessible
 *        to the GroundLink component, and thus to QGroundControl. The ParameterManager puts
 *        the parameters that are checked in into an array holding pointers to the parameters
 *        and their string identifiers (are used by QGroundControl to make parameter ids human readable)
 *
 * TODO Make parameters removable?
 * TODO Is there a more efficient way to map string IDs to values than by string comparison (hash coputation for a map takes time too)?
 */

#include "../inc/ParameterManager.h"
#include <string.h>


/**
 * \brief Constructor, returns an instance of ParameterManager.
 */
ParameterManager::ParameterManager()
{
	// Initialize float parameters structure.
	this->floats.safetyFloat = 33.3;
	for(int i=0; i<this->floats.maxNumberOfParameters; i++){
		this->floats.pointers[i] = &(this->floats.safetyFloat);
	}
	this->floats.numberOfParameters = 0;
}

/**
 * \brief Destructor.
 */
ParameterManager::~ParameterManager()
{
}

/**
 * \brief Makes a float parameter accessible to the ParameterManager.
 * \param parameterPointer - pointer to the parameter that is checked in.
 * \param textID - a string of 1-16 characters describing the parameter (e.g. "engineLaw_pGain").
 * \param textIDLength - the number of characters the textID has.
 */
void ParameterManager::checkInFloat(float* parameterPointer, char* textID, uint8_t textIDLength){
	// Check if there is still free space in the parameter array:
	if(this->floats.numberOfParameters < this->floats.maxNumberOfParameters){
		// check in pointer
		this->floats.pointers[this->floats.numberOfParameters] = parameterPointer;
		// copy text ID
		uint8_t len = textIDLength;
		// Limit the number of character to copy.
		if(len > this->floats.maxTextIdLength){
			len = this->floats.maxTextIdLength;
		}

		for(uint8_t i=0; i<len; i++){
			this->floats.textIds[this->floats.numberOfParameters][i] = textID[i];
		}
		// Fill the rest with dots to make it better to look at in QGroundControl:
		for(uint8_t i=len; i<this->floats.maxTextIdLength+1; i++){
			this->floats.textIds[this->floats.numberOfParameters][i] = ' ';
		}
		this->floats.textIdLengths[this->floats.numberOfParameters] = len;

		this->floats.numberOfParameters += 1;
	}
	return;
}

/**
 * \brief Provides the value and the text ID of the parameter with the specified index.
 * If the index exceeds the highest existing index, the function returns 0 as a value and "does not exist" as text ID
 * \param[in] index - the parameter's index.
 * \param[out] value - the parameter's value is written to this address.
 * \param[out] textID- the parameter's string/text ID is written to this char array, which has to be 16 characters long.
 */
void ParameterManager::getFloat(uint16_t index, float* value, char* textID){
	if(index < this->floats.numberOfParameters){
		*value = *(this->floats.pointers[index]);
		for(uint8_t i=0; i<this->floats.maxTextIdLength; i++){
			textID[i] = this->floats.textIds[index][i];
		}
	}else{
		*value = 0;
		textID = "does not exist";
	}
	return;
}

/**
 * \brief Sets the value of the parameter identified by the given index.
 * \param[in] index
 * \param value
 */
void ParameterManager::setFloat(uint16_t index, float value){

	if(index < this->floats.numberOfParameters){
		*(this->floats.pointers[index]) = value;
	}
	return;
}

/**
 * \brief Sets the value of the parameter identified by the given string ID.
 * \param[in] pointer to a char array holding the ID
 * \param value
 * \return the array index of the parameter.
 */
uint16_t ParameterManager::setFloat(float value, char* textID){

	uint16_t i = 0;
	while(i < this->floats.numberOfParameters){
		char* comp1 = this->floats.textIds[i];
		char* comp2 = textID;
		if(strncmp(textID, this->floats.textIds[i], this->floats.textIdLengths[i]) == 0){
			*(this->floats.pointers[i]) = value;
			return i;
		}
		i++;
	}
	return 0;
}

/**
 * \brief Returns how many parameters of type float are on the ParameterManagers's list.
 */
uint16_t ParameterManager::getNumberOfFloats(){
	return this->floats.numberOfParameters;
}
