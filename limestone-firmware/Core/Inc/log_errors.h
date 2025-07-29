/*
 * log_errors.h
 *
 *  Created on: Jul 28, 2025
 *      Author: felix
 */

// All error/status messages go in this file
// Every message MUST start with the 3 digit code, the first digit that refers to the board is added later.
// Keep all messages as short as possible, details can be added to the error lookup table

#ifndef INC_LOG_ERRORS_H_
#define INC_LOG_ERRORS_H_

// General error messages (all boards could have this happen)
#define ERR_EXAMPLE				"000 This is a general error message"

// Flight Computer error messages
#define FC_ERR_EXAMPLE			"000 This is an error message unique to the FC (e.g. TCP server error)"

// General status messages
#define STAT_EXAMPLE			"000 This is a general status message"

// Flight Computer status messages
#define FC_STAT_EXAMPLE			"000 This is a status message unique to the FC (e.g. radio thread started)"

// Error message types - used for throttling
#define ERR_TYPE_EXAMPLE		0

#endif /* INC_LOG_ERRORS_H_ */
