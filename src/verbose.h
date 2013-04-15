struct Verbose {
	explicit Verbose(unsigned char lvl = 0, std::ostream* s = 0): _stream(s)
	{
		level_ = lvl;
	}
	std::ostream* _stream;
	unsigned char level()
	{
		return level_;
	}

private:
	unsigned char level_;
};

inline Verbose operator<<(std::ostream& out, Verbose v) {
	return Verbose(v.level(), &out);
}
