#include "speaker.h"
#include <stdio.h>

/*
 * PSDK Speaker Widget for Mavic 3E.
 *
 * Uses the DJI speaker widget API (喊话器控件).
 * Supports TTS text and audio file playback.
 */

#ifdef PSDK_ENABLED
#include "dji_widget.h"

int speaker_init(void) {
    /* Speaker widget requires full registration via DjiWidget_RegSpeakerHandler.
     * TODO: Implement T_DjiWidgetSpeakerHandler callbacks for TTS/voice playback. */
    printf("[speaker] initialized (widget speaker — TODO: register handler)\n");
    return 0;
}

int speaker_play_tts(const char *text) {
    /* TODO: Speaker TTS requires DjiWidget_RegSpeakerHandler with ReceiveTtsData callback.
     * Direct API calls like DjiWidgetSpeaker_PlayTts() do not exist in PSDK 3.16.0. */
    printf("[speaker] TODO: play TTS via widget handler: %s\n", text);
    return 0;
}

int speaker_play_file(const char *file_path) {
    /* TODO: Implement via widget speaker handler ReceiveVoiceData */
    printf("[speaker] TODO: play file via widget handler: %s\n", file_path);
    return 0;
}

int speaker_set_volume(int volume) {
    /* TODO: Implement via widget speaker handler SetVolume callback */
    printf("[speaker] TODO: set volume %d via widget handler\n", volume);
    return 0;
}

int speaker_stop(void) {
    /* TODO: Implement via widget speaker handler StopPlay callback */
    printf("[speaker] TODO: stop via widget handler\n");
    return 0;
}

void speaker_cleanup(void) {}

#else /* stub */

int speaker_init(void) { printf("[speaker] stub mode\n"); return 0; }
int speaker_play_tts(const char *text) { printf("[speaker] TTS: %s\n", text); return 0; }
int speaker_play_file(const char *file_path) { return 0; }
int speaker_set_volume(int volume) { return 0; }
int speaker_stop(void) { return 0; }
void speaker_cleanup(void) {}

#endif
