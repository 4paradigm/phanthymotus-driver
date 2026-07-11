#ifndef SPEAKER_H
#define SPEAKER_H

int speaker_init(void);
int speaker_play_tts(const char *text);
int speaker_play_file(const char *file_path);
int speaker_set_volume(int volume);
int speaker_stop(void);
void speaker_cleanup(void);

#endif
