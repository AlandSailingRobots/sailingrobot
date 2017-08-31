/**
 * \file
 * Brief description. Longer description.
 *
 * \author $Author$
 * \date $Date$
 */


#ifndef __UDPCLIENT_H
#define __UDPCLIENT_H

/// a useful function to write a UDP packet prefixed by a timestamp
/// and formatted with printf semantics.
void udpwrite(const char *s,...);

#endif /* __UDPCLIENT_H */
