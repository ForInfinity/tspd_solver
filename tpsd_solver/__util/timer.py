import logging
from datetime import datetime


class Timer:
    def __init__(self, logger: logging.Logger, name: str):
        self.__logger = logger
        self.__time_list: list[tuple[str, datetime]] = []
        self.__name: str = name

    def record_event_end(self, name: str):
        self.__time_list.append((name, datetime.now()))

    def __enter__(self):
        self.record_event_end("start")
        return self

    def get_logs(self):
        logs = []
        time_list = self.__time_list.copy()
        start = time_list.pop(0)
        end = time_list.pop()
        logs.append("Timer '{}' took totally {} Seconds. Sub Tasks:".format(self.__name, end[1] - start[1]))
        last_time = start[1]
        for name, time in time_list:
            logs.append("\t-> {} spent {}".format(name, time - last_time))
            last_time = time
        logs.append("\n")
        return logs

    def print_logs(self):
        time_list = self.__time_list.copy()
        start = time_list.pop(0)
        end = time_list.pop()
        self.__logger.info("Timer {} took {}:".format(self.__name, end[1] - start[1]))
        last_time = start[1]
        for name, time in time_list:
            self.__logger.info("\t-> {} took {}".format(name, time - last_time))
            last_time = time
        self.__logger.info("\n")

    def get_time_usage(self):
        time_list = self.__time_list.copy()
        start_time = time_list.pop(0)[1]
        time_dict = {}
        for name, time in time_list:
            time_dict[f"time_usage_{name}"] = (time - start_time).total_seconds()
            start_time = time
        return time_dict

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.__time_list.append(('end', datetime.now()))
        self.print_logs()


def parse_secs_to_str(secs: float) -> str:
    return '{:0>2.0f}:{:0>2.0f}:{:0>2.0f}'.format(secs // 3600, secs % 3600 // 60, secs % 60)
