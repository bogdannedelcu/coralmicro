from commonproxy import RecentFlag, Spool, SpoolConfig, utc_now

from . import config

recent_rx = RecentFlag(config.LINK_RECENT_WINDOW_SECONDS)
spool = Spool(SpoolConfig(config.PENDING_DIR, config.SENT_DIR, config.SENT_RETENTION_DAYS))
