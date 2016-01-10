#!/bin/bash
CDATE="$(date '+%y%m%d')";
tar cvzf Logs.$CDATE.tar.gz 20*.log Logs/*
