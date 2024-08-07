Summary:	Program to measure TCP performance
Name:		tcpmon
Version:	2.3.0
Release:	1%{?dist}
License:	distributable
Group:		Applications/Internet
Vendor:		Richard Hughes-Jones <R.Hughes-Jones@man.ac.uk>
URL:		http://www.hep.man.ac.uk/~rich/net/
Source0:	https://github.com/Richard-HJ/tcpmon/tags

BuildRoot:	%(mktemp -ud %{_tmppath}/%{name}-%{version}-%{release}-XXXXXX)
BuildRequires:	libhj-devel, autoconf, automake

%description
tcpmon

%prep
%setup -q

%build
autoreconf -i
%configure --prefix=%{_prefix}
%{__make} clean
%{__make} all


%install
%{__rm} -rf "%{buildroot}"
%{__make} install DESTDIR="%{buildroot}"

%clean
%{__rm} -rf "%{buildroot}"

%files
%defattr(-,root,root,-)
%{_bindir}/tcpmon_mon
%{_bindir}/tcpmon_resp

%changelog
* Thu Feb  6 2014 Attila Bogár <attila.bogar@gmail.com>
- Fix spec and autotools

* Tue May 11 2004 Anders Waananen <waananen@nbi.dk> 
- Initial build.
