/*
 * rn52.c
 *
 * Created: 02/11/2018 18:40:52
 *  Author: swallace
 */



void rn52_init()
{
/*
    SC,200428               // Set device class: audio -> audio/video -> hi-fi audio device
    SN,Name                 // Set advertised name
    S|,0203                 // Set audio routing: S/PDIF, 48kHz sampling
    SD,04                   // Set discovery profile mask: A2DP only
    SK,04                   // Set connection mask: A2DP only
    SP,xxxx                 // Set pin code to xxxx
    ST,00                   // Set tone (beep) levels -> silent
    S%,00A6                 // Set extended features: enable reconnect at power-on, discoverable at
                            //      start-up, mute volume up/down tones, disable system tones
    SA,4                    // Set authentication: require PIN code to pair
    R,1                     // Reboot to apply settings
*/
}