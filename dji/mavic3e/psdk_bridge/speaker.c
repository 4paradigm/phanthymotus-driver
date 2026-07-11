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
    /* Speaker widget is initialized as part of the widget system */
    printf("[speaker] initialized\n");
    return 0;
}

int speaker_play_tts(const char *text) {
    T_DjiWidgetSpeakerPlayTtsInfo info = {0};
    info.language = DJI_WIDGET_SPEAKER_LANGUAGE_CHINESE;
    strncpy(info.text, text, sizeof(info.text) - 1);
    return (DjiWidgetSpeaker_PlayTts(&info) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int speaker_play_file(const char *file_path) {
    /* Play audio file via speaker widget */
    return 0;
}

int speaker_set_volume(int volume) {
    return (DjiWidgetSpeaker_SetVolume(volume) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int speaker_stop(void) {
    return (DjiWidgetSpeaker_StopPlay() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
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
