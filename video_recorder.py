#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# SPDX-License-Identifier: CC0-1.0
#
# Copyright (c) 2024 Yugo Takano (tfnh621)
#
# This file is released under the CC0 1.0 Universal (CC0 1.0) Public Domain Dedication.
# For details, see https://creativecommons.org/publicdomain/zero/1.0/

from datetime import datetime
from pathlib import Path
from shutil import which
from subprocess import check_output, Popen

#from loguru import logger


def get_best_encoder() -> str:
    output: str = check_output(['ffmpeg', '-hide_banner', '-encoders']).decode('utf-8')
    for encoder in ['av1_nvenc', 'hevc_nvenc', 'h264_nvenc', 'hevc_vaapi', 'h264_vaapi']:
        if encoder in output:
            return encoder
    #logger.debug('No suitable encoder found, using software encoder')
    return 'libx264'


# See: https://gist.github.com/tayvano/6e2d456a9897f55025e25035478a3a50
class InputSource:
    def __init__(self, infile: str):
        self.infile: str = infile

    def __str__(self) -> str:
        return f'{self.__class__.__name__}{self.to_ffmpeg_args()}'

    def to_ffmpeg_args(self) -> list[str]:
        return ['-i', self.infile]


class RawVideoSource(InputSource):
    def __init__(self, infile: str, fmt: str, codec: str, frame_size: tuple[int, int] = None, framerate: int = None):
        super().__init__(infile)
        self.fmt: str = fmt
        self.codec: str = codec
        self.screen_size: tuple[int, int] | None = frame_size  # (width, height)
        self.framerate: int | None = framerate

    def to_ffmpeg_args(self) -> list[str]:
        args = ['-f', self.fmt, '-c:v', self.codec]
        if self.screen_size is not None:
            args += ['-video_size', f'{self.screen_size[0]}x{self.screen_size[1]}']
        if self.framerate is not None:
            args += ['-framerate', str(self.framerate)]
        args += ['-i', self.infile]
        return args


class VideoRecorder:
    def __init__(self, encoder: str = None):
        self.ffmpeg_process: Popen | None = None

        if encoder is None:
            self.video_encoder = get_best_encoder()
            #logger.info(f'Using {self.video_encoder} as the encoder')
        else:
            self.video_encoder = encoder
            #logger.info(f'Forcing to use {self.video_encoder} as the encoder')

        #Path('videos').mkdir(parents=True, exist_ok=True)

    def is_recording(self) -> bool:
        return self.ffmpeg_process is not None

    def start(self, input_sources: list[InputSource], output_name: str = f'{datetime.now():%Y-%m-%d_%H-%M-%S}') -> None:
        # TODO: implement the audio recording
        if self.ffmpeg_process is not None:
            raise RuntimeError('Recording process is already running')

        ffmpeg_args = [which('ffmpeg'), '-hide_banner', '-loglevel', 'info']
        input_args = sum(map(lambda s: s.to_ffmpeg_args(), input_sources), [])
        #output_args = ['-c:v', self.video_encoder, f'videos/{output_name}.mkv']
        output_args = ['-c:v', self.video_encoder, f'{output_name}.mkv']
        if self.video_encoder.endswith('_vaapi'):
            ffmpeg_args += ['-vaapi_device', '/dev/dri/renderD128']
            output_args = ['-vf', 'format=nv12,hwupload'] + output_args

        #with open(f'logs/{output_name}.ffmpeg.log', 'wb') as log_out:
        with open(f'{output_name}.ffmpeg.log', 'wb') as log_out:
            self.ffmpeg_process = Popen(ffmpeg_args + input_args + output_args, stderr=log_out)
            #logger.debug(f'Created ffmpeg process: {self.ffmpeg_process.args}')

    def stop(self) -> None:
        if self.ffmpeg_process is None:
            #logger.debug('Trying to stop recording process, but it is not running')
            return

        self.ffmpeg_process.terminate()
        self.ffmpeg_process.communicate()
        logger.debug(f'ffmpeg has been terminated with return code {self.ffmpeg_process.returncode}')
        self.ffmpeg_process = None
